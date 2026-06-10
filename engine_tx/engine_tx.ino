/*
 * engine_tx — fake engine RPM transmitter (NMEA 2000)
 * ---------------------------------------------------
 * Standalone proof-of-concept: broadcasts PGN 127488 (Engine Parameters,
 * Rapid Update) with a fake, slowly-sweeping RPM so it shows up on the Axiom's
 * engine display. Later, replace `fakeRpm()` with a real sensor reading.
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
static const unsigned long TX_PGNS[] = { 127488L, 0 };

// Fake RPM: a slow sweep so the gauge visibly moves (proves it's live).
static double fakeRpm() {
  return 1200.0 + 800.0 * sin(millis() / 3000.0);   // ~400..2000 rpm
}

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

  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last >= 100) {    // 127488 is "rapid update" -> ~10 Hz
    last = now;
    double rpm = fakeRpm();

    tN2kMsg N2kMsg;
    SetN2kEngineParamRapid(N2kMsg, ENGINE_INSTANCE, rpm);
    NMEA2000.SendMsg(N2kMsg);

    static uint32_t logTs = 0;
    if (now - logTs >= 1000) { logTs = now; Serial.printf("RPM=%.0f\n", rpm); }
  }
}
