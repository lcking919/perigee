# Perigee — Project Context

Model rocket flight computer, RP2040 (Pico 2 W), written in C with CMake + Pico SDK.

## Architecture
- `core/` — pure logic (state machine, apogee/boost/burnout/landing detectors), no hardware dependencies, host-testable
- `drivers/` — sensor drivers (BMP388, MPU6050, ADXL375), talk through `prg_i2c_bus_t` interface
- `hal/host/` — fake I2C bus for unit testing on laptop
- `hal/pico/` — real Pico SDK backing (I2C, FatFS/SD logging, hw_config)
- `app/main.c` — actual firmware entry point
- `test/` — host-side test suite (`make test`)

## Build
- Host tests: `make test` (plain gcc, no Pico SDK needed)
- Firmware: `cmake -B build . && cmake --build build`, then flash `build/perigee.uf2` via BOOTSEL

## Current state (as of Aug 4, 2026)
- All three sensors driver-tested (fake bus) and hardware-confirmed
- FatFS/SD logging replacing LittleFS — new file per session (flight_NNN.bin)
- Two-tier logging: raw per-sensor records (flight_NNN_raw.bin) + BMP388 calib snapshot (flight_NNN_calib.bin) + derived state log (flight_NNN.bin)
- prg_flight_t state machine now wired into main.c via prg_flight_init_auto — confirmed working against FatFS backend on real hardware
- Known gap: ground-pressure reference averaging was needed to fix a ~430m altitude offset bug (fixed — see git log)

## Known issues / next steps
- Haven't tested state machine transitions (boost/apogee/landing) against real physical motion yet
- Requirements doc: docs/PRG-REQ-001-requirements.md, keep updated when adding requirements

## Conventions
- No em dashes in comments/docs (personal preference)
- Every new .c file needs adding to BOTH Makefile (TEST_SRCS) and CMakeLists.txt (add_executable) — this has caused repeated build failures
