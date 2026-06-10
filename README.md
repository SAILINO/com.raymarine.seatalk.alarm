# Raymarine SeaTalk NG / NMEA 2000 Alarm Buzzer

An ESP32-based **alarm repeater** for boats. It listens (read-only) on a
Raymarine **SeaTalk NG / NMEA 2000** backbone, decodes the common instrument
data, and sounds a **buzzer / 12 V horn** when the system raises an alarm
(e.g. an **AIS dangerous target** from a Raymarine Axiom). A small built-in
WiFi web dashboard shows live instruments and the alarm/event log on your
phone — no app, no internet needed.

> ⚠️ **Safety / disclaimer.** This is a hobby project provided "as is" (see
> [LICENSE](LICENSE)). It is **not** a certified alarm device and must not be
> relied upon for the safety of vessel or crew. It connects to the bus
> **listen-only** and never transmits, so it cannot disturb your network — but
> always verify behaviour yourself before depending on it.

## Features

- **Listen-only** NMEA 2000 reader using the ESP32's built-in TWAI/CAN
  controller (250 kbit/s) — never transmits, safe to clip onto a live bus.
- **Live instrument dashboard** over an open WiFi hotspot + captive portal:
  depth, wind (apparent/true), speed, heading, position, COG/SOG, rudder.
- **Alarms → buzzer / horn:**
  - **Shallow depth** — decoded numerically from PGN 128267 with hysteresis.
  - **Axiom alarm (generic)** — fires on any Raymarine MFD alarm (incl. AIS
    dangerous target) by detecting the autopilot's proprietary alarm burst on
    PGN 126720. See [How alarm detection works](#how-alarm-detection-works).
- **Per-alarm enable/disable toggles** on the web page, **persisted to flash
  (NVS)** so they survive reboots — handy for silencing nuisance triggers.
- **Silence / acknowledge** and **discovery** controls on the web page.
- Non-blocking horn engine: each alarm sounds a fixed number of pulses then
  stays silent (no endless honking), while a status LED blinks for the whole
  duration of the alarm. Never blocks CAN reception.
- Zero external libraries — only the ESP32 Arduino core.

## Hardware

### Parts list

| Qty | Part | Notes |
|-----|------|-------|
| 1 | **ESP32 WROOM-32** dev board | Any *classic* ESP32 (built-in TWAI/CAN controller). Not ESP8266 / not an S2 unless it has CAN. |
| 1 | **VP230 / SN65HVD230** CAN transceiver | 3.3 V module — pairs directly with the ESP32. |
| 1 | **Relay module** (1-channel) | 5 V coil with onboard driver + opto. Contacts must exceed the buzzer's V/A (any 10 A module is fine). |
| 1 | **12 V buzzer / horn** | Sized to taste. The example unit is 12 V, ≤ 200 mA. |
| 1 | **LED** (any colour) | Status indicator (blinks while an alarm is active). |
| 1 | **Resistor ~330 Ω** | Series resistor for the LED (220–470 Ω all fine). |
| 1 | **Inline fuse + holder** | On the 12 V feed to the buzzer; size to the buzzer's current. |
| 1 | **12 V → 5 V buck converter** | Powers the ESP32 (5V/VIN) + relay coil from the boat's 12 V. *Or* power via USB on the bench. |
| 1 | **NMEA 2000 / SeaTalk NG drop** | T-piece + drop cable (or a spur) to tap the backbone. |
| — | Hookup wire, Dupont jumpers, ferrules | For the low-voltage signal wiring. |
| 1 | **Enclosure** (optional) | A small sealed box if mounted in a damp location. |

> **Note:** the relay + VP230 reflect the build in this repo. A logic-level
> **MOSFET** or a suitable NPN (e.g. BC337 for loads ≤ ~0.5 A) can replace the
> relay for a low-current buzzer — set `ALARM_ACTIVE_HIGH = true` for a
> low-side transistor switch. See [WIRING.md](WIRING.md) for the full schematic.

### Why a VP230 if the ESP32 already has CAN?

Because a **CAN controller** and a **CAN transceiver** are two different things —
you need both, and the ESP32 only has the controller.

| | What it does | Signal |
|---|---|---|
| **ESP32 TWAI** (controller) | The logic: frame timing, bit-stuffing, arbitration, CRC, ACK, error handling | Single-ended **3.3 V logic** on TX/RX pins |
| **VP230** (transceiver) | The physical layer: converts that logic to/from the bus | **Differential CAN-H / CAN-L** on the wire |

The ESP32's built-in controller only exposes **logic-level TX/RX pins (0–3.3 V)**.
The NMEA 2000 / SeaTalk NG bus is a **differential pair** (CAN-H / CAN-L around
~2.5 V, with specific dominant/recessive levels, common-mode range and drive
strength) — you can't drive or read that from a GPIO. The VP230 sits in between
and does the translation, and adds the electrical robustness (proper levels to
coexist with the plotter/pilot, common-mode tolerance, ESD protection) a bare
pin doesn't have.

In short: **controller = the brain, transceiver = the mouth/ears on the wire.**
(It's the same reason a bare ESP8266 or Uno can't do this at all — they lack the
controller *and* the transceiver.)

### Wiring

📐 **Full ASCII schematic + pin map: [WIRING.md](WIRING.md).** Summary below.

**ESP32 ↔ VP230 ↔ bus:**

| ESP32 | VP230 |
|-------|-------|
| 3V3 | 3V3 |
| GND | GND (common with bus GND) |
| GPIO5 | TX / D / CTX |
| GPIO4 | RX / R / CRX |
| | CANH → backbone CAN-H |
| | CANL → backbone CAN-L |

The backbone must already be **120 Ω terminated at both ends** — tap onto an
existing terminated bus, don't add a third terminator.

**Buzzer / horn (GPIO13 → relay):** sounds 3 short pulses per alarm.

| ESP32 | Relay module |
|-------|--------------|
| GPIO13 | IN |
| 5V (VIN) | VCC |
| GND | GND (common with 12 V −) |

Load side: `12V+ → COM`, `NO → buzzer(+)`, `buzzer(−) → 12V−`. Leave **NC**
unused (silent until triggered). Most relay modules are **active-LOW**, so
`ALARM_ACTIVE_HIGH = false` (flip to `true` if the buzzer is on at idle). A
12 V transistor low-side switch (e.g. BC337 for loads ≤ ~0.5 A) works too — for
that, set `ALARM_ACTIVE_HIGH = true`.

**Status LED (GPIO22):** blinks the whole time any alarm is active.
`GPIO22 → 330 Ω → LED(+) → LED(−) → GND` (active-HIGH). Avoid strapping pins
(0, 2, 12, 15) and the RTC-crystal pins (32, 33) for the LED.

## Build & flash

1. Install the **ESP32 boards** package (Espressif) in the Arduino IDE
   (Boards Manager URL:
   `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`).
2. Select **Tools → Board → ESP32 Dev Module**.
3. Open `buzzer/buzzer.ino`, compile, and upload.
4. Open the **Serial Monitor at 115200** to watch the bus.

No libraries to install — `driver/twai.h` ships with the ESP32 core.

## Web dashboard

After boot the ESP32 starts an **open WiFi access point** (SSID set by
`AP_SSID`). Connect your phone and the dashboard pops up automatically (captive
portal), or browse to `http://192.168.4.1`.

- Live instrument cards (grey out when data goes stale).
- Red flashing banner listing active alarms; green when clear.
- Event log (alarm transitions + discovery output).
- **Silence** (acknowledge all alarms) and **Reset baseline** buttons.

## How alarm detection works

On this boat (Axiom Pro MFD + ACU-200 EV autopilot + Weatherdock easyTRX2 AIS),
the MFD does **not** emit standard NMEA 2000 Alert PGNs (126983/126985).
Instead, every alarm shows as an Axiom popup, and the autopilot (`src 204`)
emits a fixed proprietary **PGN 126720** burst (`6C 26 / 6C 27 / 6C 16`) on
each alarm event. This was confirmed empirically (present at alarm raise,
absent during quiet). The firmware triggers a generic **"Axiom alarm"** on that
burst.

This burst is **generic** — it does not encode *which* alarm fired. The
roadmap below covers decoding specific conditions directly for named alarms.

### Configuration flags (top of `buzzer.ino`)

| Flag | Purpose |
|------|---------|
| `ALARM_ACTIVE_HIGH` | `false` for an active-LOW relay (default) / `true` for a transistor or LED |
| `HORN_ON_MS` / `HORN_OFF_MS` / `ALARM_REPEATS` | Horn cadence (default 0.5 s on / 1 s off, 3 pulses) |
| `DEPTH_ON_M` / `DEPTH_OFF_M` | Shallow-depth alarm thresholds (default 2.5 m on / 3.0 m off) |
| `LED_PIN` / `LED_BLINK_MS` | Status-LED pin (default GPIO22) and blink rate |
| `DISCOVERY` / `DISCOVERY_NEW_ONLY` | Dump reassembled proprietary PGNs (reverse engineering) |
| `SIMULATE` | Cycle each alarm's pattern at boot |
| `WIFI_ENABLE` / `AP_SSID` | Web server on/off and hotspot name |

## Transmitting data (experimental)

The main `buzzer` sketch is **listen-only**. A separate proof-of-concept,
[`engine_tx/engine_tx.ino`](engine_tx/engine_tx.ino), shows the ESP32 *sending*
on the bus — it broadcasts a fake, slowly-sweeping engine RPM (**PGN 127488**)
that shows up on the Axiom's engine gauge, as the seed of a general
"NMEA → Raymarine converter."

- Uses the **NMEA2000 library** (Timo Lappalainen) + **NMEA2000_esp32** — install
  both via the Arduino Library Manager. Same VP230 wiring as the buzzer.
- It claims a source address and announces itself as an Engine node, so it
  **transmits and ACKs** (no longer passive). Replace `fakeRpm()` with a real
  sensor reading to make it a true converter.

## Roadmap

- [x] Relay-driven 12 V buzzer + status LED
- [x] Limit each alarm to a fixed number of horn pulses
- [x] Transmit PoC: publish engine RPM (PGN 127488) — `engine_tx`
- [ ] Merge TX into the main firmware (read alarms *and* publish data)
- [ ] Auto-silence the buzzer when the alarm is acknowledged on the plotter
- [ ] **Named alarms** by decoding conditions directly:
  - AIS dangerous via CPA/TCPA from PGNs 129038/129039 + own position
  - Low battery via PGN 127508
  - Off-course / wind-shift from heading + wind
- [ ] Persistent event logging to flash/SD

## License

[MIT](LICENSE)
