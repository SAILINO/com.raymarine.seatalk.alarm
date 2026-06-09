# Wiring — ASCII schematic

ESP32 WROOM-32 + VP230 CAN transceiver, relay-driven 12 V buzzer, and a status
LED. All grounds must be common. Pin numbers match the defaults at the top of
`buzzer/buzzer.ino` (`CAN_TX_PIN`, `CAN_RX_PIN`, `BUZZER_PIN`, `LED_PIN`).

## Block overview

```
   NMEA2000/SeaTalk NG  <--CANH/CANL-->  VP230  <--TX/RX/3V3/GND-->  ESP32
                                                                       |
                                          GPIO13 --> relay --> 12V buzzer
                                          GPIO22 --> status LED
                                          5V/GND <-- 12V->5V buck (or USB)
```

## 1) CAN bus  (ESP32 <-> VP230 <-> backbone)

```
        NMEA 2000 / SeaTalk NG backbone
        (120 ohm terminated at BOTH ends - do NOT add a third)
              |            |
            CAN-H        CAN-L
              |            |
        +-----+------------+--------------------------+
        |              VP230 (SN65HVD230, 3.3V)       |
        |   CANH   CANL   3V3    GND    CTX    CRX     |
        +-----------------+------+------+------+-------+
                          |      |      |      |
   ESP32 WROOM-32         |      |      |      |
        3V3   ------------+      |      |      |
        GND   -------------------+      |      |
        GPIO5  (CAN TX) ----------------+      |   (CTX = data MCU -> bus)
        GPIO4  (CAN RX) -----------------------+   (CRX = data bus -> MCU)
```

## 2) Buzzer  (ESP32 -> relay -> 12 V buzzer)

```
   ESP32              Relay module                     12 V buzzer
   GPIO13  ---------> IN
   5V/VIN  ---------> VCC                +12V --[fuse]--> COM
   GND     ---------> GND                                 NO ----> buzzer (+)
                                                          NC   (not used)
                                          buzzer (-) ---> 12V (-)

   * Relay modules are usually active-LOW -> ALARM_ACTIVE_HIGH = false
     (flip to true if the buzzer is ON at idle).
   * Use the NO contact so the buzzer is silent until an alarm fires.
   * The relay coil draws from 5V/VCC, not the GPIO - GPIO13 only drives the
     module's opto/transistor input (a few mA).
```

## 3) Status LED  (GPIO22) — blinks while any alarm is active

```
   ESP32
   GPIO22 ---[330 ohm]--->|---- GND
                         LED
   long leg (+/anode) toward the resistor; flat side (-/cathode) to GND.
   active-HIGH. Avoid strapping pins (0,2,12,15) and RTC-crystal pins (32,33).
```

## 4) Power & ground

```
   12 V battery (+) --[main fuse]--+--> 12V->5V buck --> ESP32 5V/VIN --> relay VCC
                                   |
                                   +--> relay COM (switched 12V to the buzzer)

   12 V battery (-) ---------------+--> ESP32 GND
                                   +--> VP230 GND
                                   +--> buzzer (-)
                                   ( ALL grounds common )

   Bench testing: power the ESP32 from USB instead of the buck converter.
```

## Pin map (quick reference)

```
   ESP32 pin     ->  Connects to
   ---------         ------------------------------------------------
   GPIO4         ->  VP230 CRX        (CAN RX)
   GPIO5         ->  VP230 CTX        (CAN TX)
   3V3           ->  VP230 3V3
   GPIO13        ->  Relay IN         (12 V buzzer, 3 pulses per alarm)
   GPIO22        ->  LED + 330 ohm    (status, blinks while alarm active)
   5V / VIN      ->  Relay VCC
   GND           ->  VP230 GND, Relay GND, LED cathode, 12V (-)
```

## Checklist before power-up

- [ ] Backbone already 120 ohm terminated at both ends (don't add a third).
- [ ] VP230 powered from **3.3 V**, not 5 V.
- [ ] All grounds common (ESP32, VP230, relay, 12 V supply).
- [ ] Inline **fuse** on the 12 V feed to the buzzer.
- [ ] Relay idle = buzzer **off**; if not, flip `ALARM_ACTIVE_HIGH`.
- [ ] LED on a safe GPIO (not a strapping/RTC-crystal pin).
