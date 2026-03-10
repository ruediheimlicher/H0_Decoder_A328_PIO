# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

H0_Decoder_A328_PIO is a Märklin Motorola protocol decoder for H0-scale model railway locomotives, running on an ATmega328P microcontroller. It decodes the Motorola trit-encoded DCC signal to control motor speed/direction, headlights, and optional display output.

## Build & Upload Commands

This is a PlatformIO project (Arduino framework, `atmelavr` platform):

```bash
# Build
pio run

# Upload via stk500v2 programmer
pio run --target upload

# Clean build
pio run --target clean

# Serial monitor
pio device monitor
```

Upload target: `/dev/tty.usbserial-AM0190V3` using stk500v2 protocol.

## Architecture

### Signal Decoding (main.cpp)
- **INT0 ISR** (`ISR(INT0_vect)`) on PD2 triggers on rising edges of the Motorola signal
- **Timer2 CTC ISR** measures pulse durations to distinguish HI (short) vs LO (long) impulses
- Trit decoding: each address trit is formed from two consecutive HI/LO impulses. Two complete packets must match (`lokadresseA == lokadresseB`) before data is accepted
- `LOK_ADRESSE` (default `0xCC`) is the hardcoded decoder address in ternary encoding

### Motor Control
- Motor A/B pins on `PORTC` (pins 2 and 3), controlled via PWM
- `speedlookuptable[10][15]` — 10 selectable speed profiles, each with 14 speed steps (index controlled by `speedindex`, default `8`)
- Acceleration/deceleration implemented by incrementing through the lookup table at a rate controlled by `speedchangetakt`

### Display
- **`SHOWDISPLAY` macro** (currently `0`): compile-time switch in `main.cpp` to enable/disable the EA DOGM128 SPI graphical display
- `src/display.c/h` — EA DOGM128-6 graphical LCD driver (128×64, SPI). Supports both hardware SPI (`PORTB`) and soft SPI (`PORTC`)
- `src/lcd.c/h` — HD44780 character LCD driver via 74HCT164 serial shift register on `PORTD`; **`lcd.c` is `#include`d directly into `main.cpp`**, not compiled separately
- U8g2 library (`SSD1306_128X32`) for I2C OLED (via `Wire`), declared as `u8g2` global in `main.cpp`
- `src/font.h` — raw bitmap font data for the graphical display
- `src/text.h` — `PROGMEM` string tables for display labels

### Key Hardware Pins (see defines.h)
| Function | Port | Pins |
|---|---|---|
| Motorola signal input | PD2 | INT0 |
| Motor A/B (PWM) | PORTC | 2, 3 |
| Lamp A/B | PORTC | 0, 1 |
| LCD (shift register) | PORTD | 3 (RSDS), 4 (EN), 5 (CLK) |
| SPI display (HW) | PORTB | 0 (A0), 1 (PWM), 2 (CS/SS), 3 (MOSI), 5 (SCK), 6 (RST) |
| Oscilloscope debug | PORTD | 6, 7 |

### EEPROM
- `MAX_EEPROM = 512` bytes used to store speed/configuration state
- `lasteepromaddress` tracks last written address; `saveEEPROM_Addresse` used for sequential logging

### CPU Clock
- ATmega328P configured at 8 MHz (BOD level set to 2.7V via efuse `0xFD`)

## Dependencies

- `olikraus/U8g2@^2.34.18` (declared in `platformio.ini`)
- Arduino `Wire` library (I2C for OLED)
- AVR-libc (`avr/io.h`, `avr/interrupt.h`, `avr/eeprom.h`, `avr/wdt.h`, `avr/pgmspace.h`)

## Committing Changes

**ALWAYS** use the `core:committing-changes` skill (via the Skill tool) when committing. **NEVER** commit using raw `git commit` commands directly.

## Code Conventions

- Comments and variable names are in German (Swiss/German author)
- Global state is heavily `volatile` due to ISR usage
- `PROGMEM` used for font and string data to conserve SRAM
- `display.h` contains both declarations and some definitions (including `char menubuffer[]`, `expoarray25[]`, and `volatile` position variables) — including it in multiple TUs would cause multiple-definition errors
