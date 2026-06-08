/*
 * NMEA 2000 / SeaTalk NG  ->  buzzer/horn alarm  (multi-alarm + discovery)
 * ------------------------------------------------------------------------
 * Board : ESP32 WROOM-32   CAN: VP230 transceiver, listen-only, 250 kbit/s
 *
 * Alarms handled:
 *   DIRECT (numeric, from data PGNs)        : Shallow depth (128267)
 *   NAMED  (device-raised, via Alert PGNs)  : AIS dangerous, Radar dangerous,
 *           GPS failure, Pilot low battery, Pilot off course, Pilot wind shift
 *
 * NAMED alarms arrive as NMEA2000 Alert PGNs 126983 (Alert) + 126985 (Alert
 * Text), OR as Raymarine-proprietary 126720 (older pilots). Both are
 * FAST-PACKET (multi-frame) and are reassembled below. Standard alerts are
 * matched by the text string; proprietary ones must be captured from YOUR bus
 * (turn on DISCOVERY, trigger each alarm once, send me the log).
 *
 * Output: GPIO13 (LED now; relay+horn later -> set ALARM_ACTIVE_HIGH=false).
 */

#include "driver/twai.h"
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <stdarg.h>

#define WIFI_ENABLE 1                 // 0 = disable the WiFi log server
#define AP_SSID     "ORJA-SeaTalk"   // open hotspot name (no password)
#if WIFI_ENABLE
  #include <WiFi.h>
  #include <WebServer.h>
  #include <DNSServer.h>
#endif

// ----------------------------- Config ------------------------------------
static const gpio_num_t CAN_TX_PIN = GPIO_NUM_5;
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_4;

static const int  BUZZER_PIN        = 13;
static const bool ALARM_ACTIVE_HIGH = true;   // LED test. Relay module -> false.
static const bool SELF_TEST_HONK    = true;

static const bool DISCOVERY = true;   // dump reassembled alert/proprietary PGNs to learn signatures
static const bool DISCOVERY_NEW_ONLY = true;  // log a 126720 only the FIRST time its signature appears (cuts noise)
static const uint8_t ALARM_CMD = 0x5C;        // 126720 command byte believed to carry Axiom alarms (always logged)
static const bool RAW_LOG   = false;  // dump every raw frame
static const bool SIMULATE  = false;  // true = at boot, cycle each alarm ~4s to preview its LED/horn pattern

// Direct numeric thresholds (hysteresis on/off)
static const float DEPTH_ON_M  = 2.0, DEPTH_OFF_M = 2.5;

// A NAMED alert auto-clears this many ms after the device stops re-announcing it.
static const uint32_t ALERT_TIMEOUT_MS = 15000;

// Recent events kept in RAM for the web page (defined as data here; logf() below).
static const int LOG_LINES = 60;
static const int LOG_WIDTH = 200;
static char      logBuf[LOG_LINES][LOG_WIDTH];
static int       logHead = 0, logCount = 0;

// ----------------------------- Buzzer engine -----------------------------
struct Pattern { uint16_t onMs, offMs; };
static const Pattern PAT_URGENT = {100, 100};  // depth / AIS / radar
static const Pattern PAT_MED    = {200, 200};  // gps / off-course
static const Pattern PAT_SLOW   = {200, 700};  // low batt / wind shift

// ----------------------------- Alarm table -------------------------------
// One row per alarm. Higher priority wins the buzzer when several are active.
enum AlarmId { AL_DEPTH, AL_AIS, AL_RADAR, AL_GPS, AL_PILOT_BATT,
               AL_PILOT_OC, AL_PILOT_WSHIFT, AL_AXIOM, ALARM_COUNT };

struct Alarm {
  const char* name;
  Pattern     pat;
  uint8_t     prio;
  bool        active;
  uint32_t    lastSeen;   // for auto-expire of NAMED alerts
};
static Alarm A[ALARM_COUNT] = {
  /*AL_DEPTH*/        {"DEPTH shallow",    PAT_URGENT, 100, false, 0},
  /*AL_AIS*/         {"AIS dangerous",    PAT_URGENT,  95, false, 0},
  /*AL_RADAR*/       {"RADAR dangerous",  PAT_URGENT,  95, false, 0},
  /*AL_GPS*/         {"GPS failure",      PAT_MED,     85, false, 0},
  /*AL_PILOT_BATT*/  {"PILOT low batt",   PAT_SLOW,    70, false, 0},
  /*AL_PILOT_OC*/    {"PILOT off course", PAT_MED,     80, false, 0},
  /*AL_PILOT_WSHIFT*/{"PILOT wind shift", PAT_SLOW,    60, false, 0},
  /*AL_AXIOM*/       {"AXIOM alarm",      PAT_URGENT,  90, false, 0},
};

// Print to Serial AND store in the ring buffer (with an uptime stamp).
static void logf(const char* fmt, ...) {
  char msg[256];
  va_list ap; va_start(ap, fmt);
  vsnprintf(msg, sizeof(msg), fmt, ap);
  va_end(ap);
  Serial.println(msg);
  uint32_t s = millis() / 1000;
  snprintf(logBuf[logHead], LOG_WIDTH, "%lu:%02lu  %s",
           (unsigned long)(s / 60), (unsigned long)(s % 60), msg);
  logHead = (logHead + 1) % LOG_LINES;
  if (logCount < LOG_LINES) logCount++;
}

static void raiseAlert(AlarmId id) {        // NAMED alert seen as active
  if (!A[id].active) logf(">>> ALARM ON : %s", A[id].name);
  A[id].active = true;
  A[id].lastSeen = millis();
}
static void setDirect(AlarmId id, bool on, bool off) {  // numeric w/ hysteresis
  if (!A[id].active && on)  { A[id].active = true;  logf(">>> ALARM ON : %s", A[id].name); }
  if ( A[id].active && off) { A[id].active = false; logf("<<< alarm off: %s", A[id].name); }
}

static void buzzerWrite(bool on) { digitalWrite(BUZZER_PIN, (on == ALARM_ACTIVE_HIGH) ? HIGH : LOW); }

static void updateBuzzer() {
  // Expire NAMED alerts the device stopped re-announcing.
  uint32_t now = millis();
  for (int i = 0; i < ALARM_COUNT; i++)
    if (A[i].active && A[i].lastSeen && (now - A[i].lastSeen > ALERT_TIMEOUT_MS)) {
      A[i].active = false; A[i].lastSeen = 0;
      logf("<<< alarm off (timeout): %s", A[i].name);
    }

  // Pick highest-priority active alarm.
  const Pattern* p = nullptr; int best = -1;
  for (int i = 0; i < ALARM_COUNT; i++)
    if (A[i].active && (best < 0 || A[i].prio > A[best].prio)) { best = i; p = &A[i].pat; }

  static bool phaseOn = false; static uint32_t phaseTs = 0;
  if (!p) { buzzerWrite(false); phaseOn = false; return; }
  uint16_t dur = phaseOn ? p->onMs : p->offMs;
  if (now - phaseTs >= dur) { phaseOn = !phaseOn; phaseTs = now; buzzerWrite(phaseOn); }
}

// ----------------------------- Fast-packet reassembly --------------------
struct FpSlot { bool used; uint32_t pgn; uint8_t src, seqId, total, have; uint8_t data[223]; };
static FpSlot FP[8];

static int fpFind(uint32_t pgn, uint8_t src) {
  for (int i = 0; i < 8; i++)
    if (FP[i].used && FP[i].pgn == pgn && FP[i].src == src) return i;
  return -1;
}
// Feeds one CAN frame; returns true and fills out* when a full message completes.
static bool fpFeed(uint32_t pgn, uint8_t src, const uint8_t* d, uint8_t dlc,
                   const uint8_t** outBuf, uint8_t* outLen) {
  uint8_t frame = d[0] & 0x1F, seqId = d[0] >> 5;
  int idx = fpFind(pgn, src);
  if (frame == 0) {
    if (idx < 0) for (int i = 0; i < 8; i++) if (!FP[i].used) { idx = i; break; }
    if (idx < 0) return false;                  // pool full
    FpSlot& s = FP[idx];
    s.used = true; s.pgn = pgn; s.src = src; s.seqId = seqId;
    s.total = d[1]; s.have = 0;
    for (uint8_t i = 0; i < 6 && (2 + i) < dlc && s.have < sizeof(s.data); i++)
      s.data[s.have++] = d[2 + i];
  } else {
    if (idx < 0 || FP[idx].seqId != seqId) return false;  // missed first frame
    FpSlot& s = FP[idx];
    for (uint8_t i = 0; i < 7 && (1 + i) < dlc && s.have < sizeof(s.data); i++)
      s.data[s.have++] = d[1 + i];
  }
  FpSlot& s = FP[idx];
  if (s.have >= s.total) { *outBuf = s.data; *outLen = s.total; s.used = false; return true; }
  return false;
}

// ----------------------------- Helpers -----------------------------------
static uint16_t u16(const uint8_t* d, int i){ return (uint16_t)d[i] | ((uint16_t)d[i+1]<<8); }
static int16_t  s16(const uint8_t* d, int i){ return (int16_t)u16(d, i); }
static uint32_t u32(const uint8_t* d, int i){
  return (uint32_t)d[i]|((uint32_t)d[i+1]<<8)|((uint32_t)d[i+2]<<16)|((uint32_t)d[i+3]<<24); }
static int32_t  s32(const uint8_t* d, int i){ return (int32_t)u32(d, i); }

static const float MS2KN   = 1.94384f;   // m/s -> knots
static const float RAD2DEG = 57.29578f;  // rad -> degrees

// ----------------------------- Live telemetry ----------------------------
// Latest decoded instrument values for the web dashboard. t=0 means "never
// seen"; the page greys a field out once its data goes stale.
struct Tele {
  float depth = NAN;            uint32_t depthT = 0;
  float aws = NAN, awa = NAN;   uint32_t windT = 0; bool windApp = true;
  float stw = NAN;              uint32_t stwT = 0;
  float sog = NAN, cog = NAN;   uint32_t sogT = 0;
  float hdg = NAN;              uint32_t hdgT = 0;  bool hdgMag = true;
  double lat = NAN, lon = NAN;  uint32_t posT = 0;
  float rudder = NAN;           uint32_t rudT = 0;
};
static Tele T;

// Each returns silently if a field is N/A (0xFFFF / 0x7FFFFFFF sentinels).
static void teleWind(const uint8_t* d, uint8_t len) {
  if (len < 6) return;
  uint16_t sp = u16(d, 1), an = u16(d, 3);
  if (sp != 0xFFFF) T.aws = sp * 0.01f * MS2KN;
  if (an != 0xFFFF) T.awa = an * 0.0001f * RAD2DEG;
  T.windApp = (d[5] & 0x07) == 2;     // 2 = apparent
  T.windT = millis();
}
static void teleSpeed(const uint8_t* d, uint8_t len) {
  if (len < 3) return;
  uint16_t sp = u16(d, 1);
  if (sp != 0xFFFF) { T.stw = sp * 0.01f * MS2KN; T.stwT = millis(); }
}
static void telePos(const uint8_t* d, uint8_t len) {
  if (len < 8) return;
  int32_t la = s32(d, 0), lo = s32(d, 4);
  if ((uint32_t)la != 0x7FFFFFFF) { T.lat = la * 1e-7; T.lon = lo * 1e-7; T.posT = millis(); }
}
static void teleCogSog(const uint8_t* d, uint8_t len) {
  if (len < 6) return;
  uint16_t c = u16(d, 2), s = u16(d, 4);
  if (c != 0xFFFF) T.cog = c * 0.0001f * RAD2DEG;
  if (s != 0xFFFF) T.sog = s * 0.01f * MS2KN;
  T.sogT = millis();
}
static void teleHeading(const uint8_t* d, uint8_t len) {
  if (len < 8) return;
  uint16_t h = u16(d, 1);
  if (h != 0xFFFF) { T.hdg = h * 0.0001f * RAD2DEG; T.hdgMag = (d[7] & 0x03) == 1; T.hdgT = millis(); }
}
static void teleRudder(const uint8_t* d, uint8_t len) {
  if (len < 6) return;
  int16_t r = s16(d, 4);
  if ((uint16_t)r != 0x7FFF) { T.rudder = r * 0.0001f * RAD2DEG; T.rudT = millis(); }
}

// Extract the longest printable-ASCII run from a payload (for Alert Text).
static void extractText(const uint8_t* p, uint8_t len, char* out, size_t outSz) {
  size_t bestStart = 0, bestLen = 0, curStart = 0, curLen = 0;
  for (uint8_t i = 0; i < len; i++) {
    if (p[i] >= 0x20 && p[i] <= 0x7E) {
      if (curLen == 0) curStart = i;
      curLen++;
      if (curLen > bestLen) { bestLen = curLen; bestStart = curStart; }
    } else curLen = 0;
  }
  size_t n = bestLen < outSz - 1 ? bestLen : outSz - 1;
  memcpy(out, p + bestStart, n); out[n] = 0;
}

// ----------------------------- NAMED alert matching ----------------------
// Map an alert's text to one of our alarm rows. Case-insensitive substring.
// Extend/adjust once we see your gear's actual wording in the DISCOVERY log.
static void matchNamedAlert(const char* textRaw) {
  char t[64]; size_t n = 0;
  for (const char* s = textRaw; *s && n < sizeof(t) - 1; s++) t[n++] = tolower(*s);
  t[n] = 0;
  bool hit = strstr(t,"ais") && (strstr(t,"danger")||strstr(t,"cpa")||strstr(t,"collision"));
  if (hit) raiseAlert(AL_AIS);
  if (strstr(t,"radar") && (strstr(t,"danger")||strstr(t,"guard")||strstr(t,"target"))) raiseAlert(AL_RADAR);
  if (strstr(t,"gps")||strstr(t,"gnss")||strstr(t,"no position")||strstr(t,"position lost")) raiseAlert(AL_GPS);
  if (strstr(t,"low batt")||strstr(t,"battery")) raiseAlert(AL_PILOT_BATT);
  if (strstr(t,"off course")||strstr(t,"offcourse")) raiseAlert(AL_PILOT_OC);
  if (strstr(t,"wind shift")||strstr(t,"windshift")||strstr(t,"wind change")) raiseAlert(AL_PILOT_WSHIFT);
  if (strstr(t,"shallow")||strstr(t,"depth")) raiseAlert(AL_DEPTH);
}

// ----------------------------- Novelty filter ----------------------------
// Remembers which proprietary message "signatures" we've already logged, so
// repeating chatter is shown once and a NEW message (e.g. an alarm) stands out.
static uint32_t seenSig[200];
static int      seenCount = 0;
// key = src + the message-type id bytes (d[2..4]); data bytes are ignored.
static uint32_t sigKey(uint8_t src, const uint8_t* d, uint8_t len) {
  uint8_t a = len > 2 ? d[2] : 0, b = len > 3 ? d[3] : 0, c = len > 4 ? d[4] : 0;
  return ((uint32_t)src << 24) | ((uint32_t)a << 16) | ((uint32_t)b << 8) | c;
}
static bool seenAddNew(uint32_t k) {            // true if this signature is new
  for (int i = 0; i < seenCount; i++) if (seenSig[i] == k) return false;
  if (seenCount < (int)(sizeof(seenSig) / sizeof(seenSig[0]))) seenSig[seenCount++] = k;
  return true;
}
static void seenReset() { seenCount = 0; }

// ----------------------------- PGN handlers ------------------------------
static void onDepth(const uint8_t* d, uint8_t len) {
  if (len < 5) return;
  uint32_t raw = u32(d, 1);
  if (raw == 0xFFFFFFFF) return;
  float depth = raw * 0.01f;
  T.depth = depth; T.depthT = millis();
  setDirect(AL_DEPTH, depth <= DEPTH_ON_M, depth > DEPTH_OFF_M);
}

// Standard Alert Text (126985): byte0 type/cat, then ... , trailing ASCII text.
static void onAlertText(uint8_t src, const uint8_t* d, uint8_t len) {
  uint8_t type = d[0] & 0x0F, cat = (d[0] >> 4) & 0x0F;
  uint16_t alertId = len >= 5 ? u16(d, 3) : 0;
  char text[64]; extractText(d, len, text, sizeof(text));
  if (DISCOVERY) logf("[ALERT-TEXT src=%u] type=%u cat=%u id=%u text=\"%s\"", src, type, cat, alertId, text);
  matchNamedAlert(text);
}

// Append "AA BB CC " hex of d[0..len) into buf at offset *n (bounded).
static void appendHex(char* buf, size_t sz, int* n, const uint8_t* d, uint8_t len) {
  for (uint8_t i = 0; i < len && *n < (int)sz - 4; i++)
    *n += snprintf(buf + *n, sz - *n, "%02X ", d[i]);
}

// Standard Alert (126983): announce; details parsed once we see your data.
static void onAlert(uint8_t src, const uint8_t* d, uint8_t len) {
  if (!DISCOVERY) return;
  uint8_t type = d[0] & 0x0F, cat = (d[0] >> 4) & 0x0F;
  uint16_t alertId = len >= 5 ? u16(d, 3) : 0;
  char buf[256]; int n = snprintf(buf, sizeof(buf), "[ALERT src=%u] type=%u cat=%u id=%u raw=", src, type, cat, alertId);
  appendHex(buf, sizeof(buf), &n, d, len);
  logf("%s", buf);
}

// Raymarine proprietary (126720): dump so we can decode YOUR pilot's alarms.
static void onProprietary(uint8_t src, const uint8_t* d, uint8_t len) {
  if (!DISCOVERY) return;
  bool isAlarm = (len > 2 && d[2] == ALARM_CMD);          // candidate alarm msg: always log
  bool isNew   = seenAddNew(sigKey(src, d, len));
  if (DISCOVERY_NEW_ONLY && !isNew && !isAlarm) return;   // suppress repeated routine chatter
  const char* tag = isAlarm ? "*** ALARM? " : (isNew ? "NEW " : "");
  char buf[256];
  int n = snprintf(buf, sizeof(buf), "%s[126720 src=%u len=%u] ", tag, src, len);
  appendHex(buf, sizeof(buf), &n, d, len);
  char text[64]; extractText(d, len, text, sizeof(text));
  if (text[0]) snprintf(buf + n, sizeof(buf) - n, "| ascii=\"%s\"", text);
  logf("%s", buf);
}

// The Raymarine pilot (src 204) emits this fixed 6C burst on any Axiom alarm
// event (confirmed: present at alarm raise, absent for 3 min of quiet). It does
// not say WHICH alarm, so we treat it as a generic "Axiom alarm" trigger.
static void checkAxiomAlarm(uint8_t src, const uint8_t* d, uint8_t len) {
  if (src == 204 && len >= 4 && d[2] == 0x6C &&
      (d[3] == 0x26 || d[3] == 0x27 || d[3] == 0x16)) {
    raiseAlert(AL_AXIOM);
  }
}

// ----------------------------- Simulate (pattern preview) ----------------
static void simulateAlarms() {
  Serial.println(F("SIMULATE: previewing each alarm pattern (4s each)..."));
  for (int i = 0; i < ALARM_COUNT; i++) {
    Serial.printf("  -> %s\n", A[i].name);
    A[i].active = true;
    uint32_t until = millis() + 4000;
    while (millis() < until) { updateBuzzer(); delay(5); }   // keep pattern ticking
    A[i].active = false;
    buzzerWrite(false);
    delay(600);
  }
  Serial.println(F("SIMULATE done."));
}

// ----------------------------- WiFi log server ---------------------------
#if WIFI_ENABLE
static WebServer server(80);
static DNSServer dns;

// Self-contained page: polls /data every 1.5s, shows active alarms + log.
static const char PAGE[] PROGMEM = R"HTML(<!doctype html><html><head>
<meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>SeaTalk Alarm</title><style>
body{font-family:system-ui,Arial;margin:0;background:#0b1f2a;color:#e8f0f4}
header{padding:14px 16px;background:#0e2a38;font-size:18px;font-weight:600}
#b{padding:12px 16px;font-weight:700}.ok{background:#13502a}
.al{background:#7a1420;animation:bl 1s steps(1) infinite}
@keyframes bl{50%{background:#3a0a10}}
#dash{display:grid;grid-template-columns:repeat(auto-fill,minmax(120px,1fr));gap:8px;padding:10px 12px}
.c{background:#0e2a38;border-radius:8px;padding:8px 10px}.c.st{opacity:.35}
.cl{font-size:11px;color:#8fb0c0;text-transform:uppercase;letter-spacing:.5px}
.cv{font-size:20px;font-weight:600;margin-top:2px}
h2{font-size:12px;color:#8fb0c0;margin:6px 12px 0;text-transform:uppercase;letter-spacing:.5px}
button{background:#1c4a5e;color:#e8f0f4;border:0;border-radius:6px;padding:6px 12px;font-size:13px;float:right;margin-top:-2px}
#log{padding:4px 12px;font-family:monospace;font-size:12px}
.r{padding:3px 0;border-bottom:1px solid #16323f;white-space:pre-wrap}
</style></head><body><header>&#9875; SeaTalk / NMEA2000
<button onclick="fetch('/mute')">Silence</button>
<button onclick="fetch('/reset')">Reset baseline</button></header>
<div id=b class=ok>connecting&#8230;</div>
<div id=dash></div><h2>Event log</h2><div id=log></div><script>
function esc(s){return s.replace(/[<&]/g,c=>c=='<'?'&lt;':'&amp;')}
async function t(){try{
 const x=(await (await fetch('/data',{cache:'no-store'})).text()).split('\n');
 const a=x[0].slice(7).split(',').filter(s=>s.length),b=document.getElementById('b');
 if(a.length){b.className='al';b.textContent='⚠ '+a.join('   •   ')}
 else{b.className='ok';b.textContent='All clear'}
 let i=1,dash=[];
 for(;i<x.length&&x[i].slice(0,2)=='D:';i++){const p=x[i].slice(2).split('|');dash.push(p)}
 if(x[i]=='===')i++;
 document.getElementById('dash').innerHTML=dash.map(p=>
  '<div class="c'+((+p[2])>10?' st':'')+'"><div class=cl>'+esc(p[0])+
  '</div><div class=cv>'+esc(p[1])+'</div></div>').join('');
 document.getElementById('log').innerHTML=x.slice(i).filter(s=>s.length)
  .map(s=>'<div class=r>'+esc(s)+'</div>').join('')
}catch(e){}}
setInterval(t,1500);t()</script></body></html>)HTML";

static void handleRoot() { server.send_P(200, "text/html", PAGE); }

static void handleData() {
  uint32_t now = millis();
  String out = "ACTIVE:";
  bool first = true;
  for (int i = 0; i < ALARM_COUNT; i++)
    if (A[i].active) { if (!first) out += ","; out += A[i].name; first = false; }
  out += "\n";

  // Dashboard rows: "D:label|value|ageSeconds"
  char b[48];
  auto add = [&](const char* label, uint32_t t, const char* val) {
    if (!t) return;
    out += "D:"; out += label; out += "|"; out += val; out += "|";
    out += String((now - t) / 1000); out += "\n";
  };
  if (!isnan(T.depth)) { snprintf(b, sizeof b, "%.1f m", T.depth); add("Depth", T.depthT, b); }
  if (!isnan(T.aws))   { snprintf(b, sizeof b, "%.1f kn  %.0f\xC2\xB0 %s", T.aws,
                                  isnan(T.awa) ? 0 : T.awa, T.windApp ? "App" : "True"); add("Wind", T.windT, b); }
  if (!isnan(T.stw))   { snprintf(b, sizeof b, "%.1f kn", T.stw);           add("Speed STW", T.stwT, b); }
  if (!isnan(T.sog))   { snprintf(b, sizeof b, "%.1f kn", T.sog);           add("SOG", T.sogT, b); }
  if (!isnan(T.cog))   { snprintf(b, sizeof b, "%.0f\xC2\xB0", T.cog);      add("COG", T.sogT, b); }
  if (!isnan(T.hdg))   { snprintf(b, sizeof b, "%.0f\xC2\xB0 %s", T.hdg, T.hdgMag ? "M" : "T"); add("Heading", T.hdgT, b); }
  if (!isnan(T.lat))   { snprintf(b, sizeof b, "%.4f%c  %.4f%c", fabs(T.lat), T.lat >= 0 ? 'N' : 'S',
                                  fabs(T.lon), T.lon >= 0 ? 'E' : 'W'); add("Position", T.posT, b); }
  if (!isnan(T.rudder)){ snprintf(b, sizeof b, "%+.0f\xC2\xB0", T.rudder); add("Rudder", T.rudT, b); }

  out += "===\n";
  for (int k = 0; k < logCount; k++) {           // newest first
    int idx = ((logHead - 1 - k) % LOG_LINES + LOG_LINES) % LOG_LINES;
    out += logBuf[idx]; out += "\n";
  }
  server.send(200, "text/plain", out);
}

static void handleReset() {
  seenReset();
  logf("-- baseline reset: now logging only NEW proprietary signatures --");
  server.send(200, "text/plain", "ok");
}

static void handleMute() {                        // user acknowledges: clear all alarms
  for (int i = 0; i < ALARM_COUNT; i++) { A[i].active = false; A[i].lastSeen = 0; }
  buzzerWrite(false);
  logf("-- silenced by user --");
  server.send(200, "text/plain", "ok");
}

static void setupWifi() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID);                           // open AP, no password
  IPAddress ip = WiFi.softAPIP();
  dns.start(53, "*", ip);                         // captive portal: any host -> us
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/reset", handleReset);               // clear learned signatures (baseline)
  server.on("/mute", handleMute);                 // acknowledge / silence all alarms
  server.onNotFound(handleRoot);                  // any URL shows the page (pops captive portal)
  server.begin();
  logf("WiFi AP '%s' up -> connect, browse http://%s/", AP_SSID, ip.toString().c_str());
}
#endif

// ----------------------------- Setup / loop ------------------------------
static uint32_t frameCount = 0, lastStatus = 0;

void setup() {
  Serial.begin(115200);
  digitalWrite(BUZZER_PIN, ALARM_ACTIVE_HIGH ? LOW : HIGH);
  pinMode(BUZZER_PIN, OUTPUT);
  buzzerWrite(false);
  delay(300);
  Serial.println(F("\n=== NMEA2000 multi-alarm buzzer (listen-only) ==="));

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_LISTEN_ONLY);
  g.rx_queue_len = 32;   // deeper queue so serving a web page can't drop CAN frames
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK || twai_start() != ESP_OK) {
    Serial.println(F("ERROR: TWAI init failed"));
    while (true) { buzzerWrite(true); delay(80); buzzerWrite(false); delay(80); }
  }
  if (SELF_TEST_HONK) { buzzerWrite(true); delay(150); buzzerWrite(false); }
  if (SIMULATE) simulateAlarms();
#if WIFI_ENABLE
  setupWifi();
#endif
  logf("Running. DISCOVERY on -> trigger each alarm once and capture the log.");
}

void loop() {
  twai_message_t msg;
  if (twai_receive(&msg, pdMS_TO_TICKS(5)) == ESP_OK) {
    frameCount++;
    uint32_t id = msg.identifier;
    uint8_t  pf = (id >> 16) & 0xFF, ps = (id >> 8) & 0xFF, src = id & 0xFF, dp = (id >> 24) & 0x01;
    uint32_t pgn = (pf < 240) ? (((uint32_t)dp<<16)|((uint32_t)pf<<8))
                              : (((uint32_t)dp<<16)|((uint32_t)pf<<8)|ps);

    if (RAW_LOG) {
      Serial.printf("PGN=%-6lu src=%-3u ", (unsigned long)pgn, src);
      for (int i=0;i<msg.data_length_code;i++) Serial.printf("%02X ", msg.data[i]);
      Serial.println();
    }

    const uint8_t* m; uint8_t mlen;
    switch (pgn) {
      // --- single-frame data PGNs ---
      case 128267: onDepth(msg.data, msg.data_length_code);     break;  // Water Depth
      case 130306: teleWind(msg.data, msg.data_length_code);    break;  // Wind Data
      case 128259: teleSpeed(msg.data, msg.data_length_code);   break;  // Speed, Water Ref
      case 129025: telePos(msg.data, msg.data_length_code);     break;  // Position, Rapid
      case 129026: teleCogSog(msg.data, msg.data_length_code);  break;  // COG & SOG, Rapid
      case 127250: teleHeading(msg.data, msg.data_length_code); break;  // Vessel Heading
      case 127245: teleRudder(msg.data, msg.data_length_code);  break;  // Rudder

      // --- fast-packet PGNs: reassemble first ---
      case 126985: if (fpFeed(pgn, src, msg.data, msg.data_length_code, &m, &mlen)) onAlertText(src, m, mlen); break;
      case 126983: if (fpFeed(pgn, src, msg.data, msg.data_length_code, &m, &mlen)) onAlert(src, m, mlen);     break;
      case 126720: if (fpFeed(pgn, src, msg.data, msg.data_length_code, &m, &mlen)) { checkAxiomAlarm(src, m, mlen); onProprietary(src, m, mlen); } break;
      default: break;
    }
  }

  updateBuzzer();

#if WIFI_ENABLE
  dns.processNextRequest();
  server.handleClient();
#endif

  uint32_t now = millis();
  if (now - lastStatus >= 2000) {
    lastStatus = now;
    Serial.printf("[%6lu fr] active:", (unsigned long)frameCount);
    bool any = false;
    for (int i = 0; i < ALARM_COUNT; i++) if (A[i].active) { Serial.printf(" %s |", A[i].name); any = true; }
    Serial.println(any ? "" : " (all clear)");
  }
}
