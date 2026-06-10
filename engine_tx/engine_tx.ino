/*
 * engine_tx — fake engine data transmitter (NMEA 2000)
 * ----------------------------------------------------
 * Standalone proof-of-concept: broadcasts fake, slowly-sweeping engine values
 * so they show up on the Axiom's engine display. Replace the fake*() helpers
 * with real sensor readings later.
 *   PGN 127488 (Rapid Update) : RPM
 *   PGN 127489 (Dynamic)      : oil pressure, oil temp, coolant temp
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
static const unsigned long TX_PGNS[] = { 127488L, 127489L, 0 };

// ALARM_TEST: send OVER-LIMIT values + warning flags to make the plotter raise
// engine alarms (coolant >130 C, oil pressure ~14 psi). false = normal sweeps.
static const bool ALARM_TEST = true;

// Fake values: normal demo = slow sweeps; ALARM_TEST = fixed out-of-range.
static double fakeRpm()        { return ALARM_TEST ? 2000.0 : 1200.0 + 800.0 * sin(millis() / 3000.0); }
static double fakeCoolantC()   { return ALARM_TEST ? 135.0  : 82.0   + 6.0   * sin(millis() / 9000.0); }  // >130 = over-temp
static double fakeOilC()       { return 95.0 + 5.0 * sin(millis() / 11000.0); }                          // oil temp stays normal
static double fakeOilPressPa() { return ALARM_TEST ? 1.0e5  : (4.0 + 0.3 * sin(millis() / 7000.0)) * 1e5; } // 1 bar (~14.5 psi) < 20

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n=== engine_tx: fake RPM over PGN 127488 ==="));

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
}
