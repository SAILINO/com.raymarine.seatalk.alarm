/*
 * engine_tx — fake engine data transmitter (NMEA 2000)
 * ----------------------------------------------------
 * Standalone proof-of-concept: broadcasts fake, slowly-sweeping engine values
 * so they show up on the Axiom's engine display. Replace the fake*() helpers
 * with real sensor readings later.
 *   PGN 127488 (Rapid Update) : RPM
 *   PGN 127489 (Dynamic)      : oil pressure, oil temp, coolant temp
 *   PGN 127508 (Battery)      : voltage (random 10..14 V) + current
 *   PGN 127505 (Fluid Level)  : fuel / fresh water / black water tanks
 *   PGN 129025/129026/129029  : GPS position, COG/SOG, GNSS fix
 *   PGN 130306 (Wind)         : apparent wind speed + angle
 *
 * Board : ESP32 WROOM-32   CAN: VP230 transceiver (same wiring as the buzzer)
 *   ESP32 GPIO5 -> VP230 CTX (TX),  GPIO4 -> VP230 CRX (RX),  3V3, GND, CANH/CANL
 *
 * Unlike the buzzer (listen-only), this node TRANSMITS and ACKs on the bus.
 * It claims a source address and announces itself as an Engine device, so it
 * behaves as a proper NMEA 2000 node.
 *
 * Required libraries (Arduino IDE -> Library Manager, both by Timo Lappalainen):
 *   - "NMEA2000 Library"
 *   - "NMEA2000_esp32"
 */

#include <Arduino.h>
#include <math.h>

// CAN pins must be defined BEFORE including NMEA2000_CAN.h.
#define ESP32_CAN_TX_PIN GPIO_NUM_5
#define ESP32_CAN_RX_PIN GPIO_NUM_4

#include <NMEA2000_CAN.h>   // auto-selects the ESP32 TWAI backend
#include <N2kMessages.h>

static const uint8_t ENGINE_INSTANCE = 0;     // 0 = single / port engine
static const uint8_t NODE_SOURCE_ADDR = 22;   // starting address (auto-claims if taken)

// PGNs we transmit (announced to the bus).
//   127488 = Engine Parameters, Rapid Update (RPM)
//   127489 = Engine Parameters, Dynamic     (oil temp, coolant temp, ...)
//   127508 = Battery Status                 (voltage)
//   127505 = Fluid Level                    (tanks)
//   129025/129026/129029 = Position rapid / COG-SOG rapid / GNSS fix
//   130306 = Wind Data                      (apparent)
static const unsigned long TX_PGNS[] = {
  127488L, 127489L, 127508L, 127505L, 129025L, 129026L, 129029L, 130306L, 0
};

// ALARM_TEST: send OVER-LIMIT values + warning flags to make the plotter raise
// engine alarms (coolant >130 C, oil pressure ~14 psi). false = normal sweeps.
static const bool ALARM_TEST = true;

// Fake values: normal demo = slow sweeps; ALARM_TEST = fixed out-of-range.
static double fakeRpm()        { return ALARM_TEST ? 2000.0 : 1200.0 + 800.0 * sin(millis() / 3000.0); }
static double fakeCoolantC()   { return ALARM_TEST ? 135.0  : 82.0   + 6.0   * sin(millis() / 9000.0); }  // >130 = over-temp
static double fakeOilC()       { return 95.0 + 5.0 * sin(millis() / 11000.0); }                          // oil temp stays normal
static double fakeOilPressPa() { return ALARM_TEST ? 1.0e5  : (4.0 + 0.3 * sin(millis() / 7000.0)) * 1e5; } // 1 bar (~14.5 psi) < 20
static double fakeBatteryV()   { return random(1000, 1401) / 100.0; }      // random 10.00 .. 14.00 V
static double fakeBatteryA()   { return random(0, 5001) / 100.0 - 20.0; }  // random -20.0 .. +30.0 A

static const double   KN2MS     = 0.514444;   // knots -> m/s
static const uint16_t DAYS_1970 = 20614;      // fake GNSS date (2026-06-10)

// GPS: a position slowly circling off the Ligurian coast, with COG/SOG.
static double fakeLat()    { return 43.1000 + 0.010 * sin(millis() / 60000.0); }
static double fakeLon()    { return 7.4000  + 0.010 * cos(millis() / 60000.0); }
static double fakeCogDeg() { return 90.0 + 30.0 * sin(millis() / 30000.0); }   // ~60..120
static double fakeSogKn()  { return 5.5; }
// Wind (apparent).
static double fakeWindKn()     { return 8.0  + 4.0  * sin(millis() / 8000.0); }   // ~4..12 kn
static double fakeWindAngDeg()  { return 40.0 + 20.0 * sin(millis() / 10000.0); } // ~20..60 deg

// Fake tanks: instance, fluid type, base level %, sweep amplitude %, capacity L.
struct FakeTank { uint8_t instance; tN2kFluidType type; double base, amp, capacityL; };
static const FakeTank TANKS[] = {
  { 0, N2kft_Fuel,       65.0, 10.0, 200.0 },   // fuel
  { 1, N2kft_Water,      50.0, 15.0, 150.0 },   // fresh water
  { 2, N2kft_BlackWater, 30.0, 10.0,  80.0 },   // waste / black water
};

void setup() {
  Serial.begin(115200);
  delay(300);
  randomSeed(esp_random());
  Serial.println(F("\n=== engine_tx: fake engine / battery / tank data ==="));

  // Identify ourselves on the bus.
  NMEA2000.SetProductInformation(
      "SAILINO-ENG-1",     // model serial code
      100,                  // product code
      "Sailino Engine TX",  // model id
      "1.0.0 (2026)",       // sw version
      "Sailino N2K");       // model version
  // UniqueNumber, DeviceFunction=160 (Engine), DeviceClass=50 (Propulsion),
  // ManufacturerCode=2046 (placeholder/dev), IndustryGroup=4 (Marine).
  NMEA2000.SetDeviceInformation(1, 160, 50, 2046);

  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, NODE_SOURCE_ADDR);
  NMEA2000.ExtendTransmitMessages(TX_PGNS);
  NMEA2000.EnableForward(false);              // no debug echo to Serial
  if (!NMEA2000.Open()) {
    Serial.println(F("ERROR: NMEA2000 open failed"));
    while (true) delay(1000);
  }
  Serial.println(F("Node up. Transmitting fake RPM at 10 Hz..."));
}

void loop() {
  NMEA2000.ParseMessages();   // MUST run often: address claim, ACKs, tx queue
  uint32_t now = millis();

  // PGN 127488 - RPM (rapid update, ~10 Hz)
  static uint32_t lastRapid = 0;
  if (now - lastRapid >= 100) {
    lastRapid = now;
    tN2kMsg m;
    SetN2kEngineParamRapid(m, ENGINE_INSTANCE, fakeRpm());
    NMEA2000.SendMsg(m);
  }

  // PGN 127489 - oil pressure / oil temp / coolant temp (dynamic, ~1 Hz)
  static uint32_t lastDyn = 0;
  if (now - lastDyn >= 1000) {
    lastDyn = now;
    double oilP = fakeOilPressPa(), oilT = fakeOilC(), coolT = fakeCoolantC();
    // Discrete warning flags (Status 1): bit1 = OverTemperature, bit2 = LowOilPressure.
    // Set them in ALARM_TEST so the plotter raises the warning explicitly, not
    // just via its configured numeric thresholds.
    tN2kEngineDiscreteStatus1 st1(ALARM_TEST ? 0x0006 : 0x0000);
    tN2kMsg m;
    // All trailing args given explicitly to disambiguate the overloaded helper.
    SetN2kEngineDynamicParam(m, ENGINE_INSTANCE,
        oilP,                         // oil pressure (Pa)
        CToKelvin(oilT),              // oil temperature
        CToKelvin(coolT),             // coolant temperature
        N2kDoubleNA,                  // alternator voltage (not faked)
        N2kDoubleNA,                  // fuel rate
        N2kDoubleNA,                  // engine hours
        N2kDoubleNA,                  // coolant pressure
        N2kDoubleNA,                  // fuel pressure
        N2kInt8NA,                    // engine load
        N2kInt8NA,                    // engine torque
        st1,                          // status 1 (warning bits)
        tN2kEngineDiscreteStatus2(0));// status 2
    NMEA2000.SendMsg(m);
    Serial.printf("RPM=%.0f  oilP=%.1fbar  oilT=%.0fC  coolant=%.0fC%s\n",
                  fakeRpm(), oilP / 1e5, oilT, coolT, ALARM_TEST ? "  [ALARM TEST]" : "");
  }

  // PGN 127508 - battery status for each bank (random 10..14 V), ~1.5 s
  static const uint8_t BATTERIES[] = { 0, 1 };   // 0 = house, 1 = start
  static uint32_t lastBat = 0;
  if (now - lastBat >= 1500) {
    lastBat = now;
    for (unsigned i = 0; i < sizeof(BATTERIES) / sizeof(BATTERIES[0]); i++) {
      double v = fakeBatteryV(), a = fakeBatteryA();   // independent random values per bank
      tN2kMsg m;
      SetN2kDCBatStatus(m, BATTERIES[i], v, a, N2kDoubleNA, 0xff);   // voltage, current, temp
      NMEA2000.SendMsg(m);
      Serial.printf("battery[%u]=%.2f V  %.1f A\n", BATTERIES[i], v, a);
    }
  }

  // PGN 127505 - fluid level for each fake tank, ~2.5 s
  static uint32_t lastTank = 0;
  if (now - lastTank >= 2500) {
    lastTank = now;
    for (unsigned i = 0; i < sizeof(TANKS) / sizeof(TANKS[0]); i++) {
      const FakeTank &tk = TANKS[i];
      double level = tk.base + tk.amp * sin(millis() / 13000.0 + tk.instance);
      tN2kMsg m;
      SetN2kFluidLevel(m, tk.instance, tk.type, level, tk.capacityL);
      NMEA2000.SendMsg(m);
    }
  }

  // PGN 129025 position + 129026 COG/SOG (rapid, ~4 Hz)
  static uint32_t lastPos = 0;
  if (now - lastPos >= 250) {
    lastPos = now;
    tN2kMsg mp;
    SetN2kLatLonRapid(mp, fakeLat(), fakeLon());
    NMEA2000.SendMsg(mp);
    tN2kMsg mc;
    SetN2kCOGSOGRapid(mc, 0xff, N2khr_true, DegToRad(fakeCogDeg()), fakeSogKn() * KN2MS);
    NMEA2000.SendMsg(mc);
  }

  // PGN 129029 GNSS fix (full position with sats/HDOP), ~1 Hz
  static uint32_t lastGnss = 0;
  if (now - lastGnss >= 1000) {
    lastGnss = now;
    double secs = fmod(millis() / 1000.0, 86400.0);   // fake UTC seconds-since-midnight
    tN2kMsg m;
    SetN2kGNSS(m, 0xff, DAYS_1970, secs, fakeLat(), fakeLon(), 0.0 /*alt*/,
               N2kGNSSt_GPS, N2kGNSSm_GNSSfix, 11 /*sats*/, 0.9 /*HDOP*/, 1.5 /*PDOP*/, 0.0 /*geoidal*/);
    NMEA2000.SendMsg(m);
  }

  // PGN 130306 apparent wind (~4 Hz)
  static uint32_t lastWind = 0;
  if (now - lastWind >= 250) {
    lastWind = now;
    tN2kMsg m;
    SetN2kWindSpeed(m, 0xff, fakeWindKn() * KN2MS, DegToRad(fakeWindAngDeg()), N2kWind_Apparent);
    NMEA2000.SendMsg(m);
  }
}
