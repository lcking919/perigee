# Perigee — Requirements

## Verification methods

| Code | Method |
|---|---|
| T | Test — exercise the article and measure |
| A | Analysis — calculation or simulation |
| I | Inspection — examine the design or code |
| D | Demonstration — observe correct operation |

## Flight software

**PRG-FSW-030** The flight computer shall detect apogee within 200 ms of
true apogee.

*Rationale:* Bounds the error in reported apogee altitude, and in a later
phase would bound deployment timing error.

*Verification:* T — replay of synthetic and recorded flight profiles.

**PRG-FSW-020** The flight computer shall transition from `ARMED` to
`BOOST` upon detecting sustained acceleration exceeding 3g for not less
than 100 ms.

*Rationale:* Threshold and duration together reject handling transients
and pad bumps while triggering reliably on ignition.

*Verification:* T — test_boost_detects_ignition and
test_boost_ignores_a_single_jolt in test_apogee.c.

**PRG-FSW-025** The flight computer shall transition from `BOOST` to
`COAST` upon detecting acceleration sustained at or below 1.5g for not
less than 100 ms.

*Rationale:* Marks motor burnout. Mirrors PRG-FSW-020's threshold-and-hold
logic in reverse — a single low reading (e.g. a brief thrust dip during an
irregular burn) should not be mistaken for burnout.

*Verification:* T — test_state_machine_full_flight in test_apogee.c,
which exercises the full ARMED->BOOST->COAST->DESCENT sequence.

**PRG-FSW-065** The flight state machine shall correctly sequence through
`ARMED`, `BOOST`, `COAST`, and `DESCENT` for a representative flight
profile, with each detector consulted only in its corresponding state.

*Rationale:* Individual detector correctness does not guarantee correct
orchestration — PRG-STATE_BOOST briefly had no exit condition at all
despite every individual detector working, and was only caught by an
end-to-end test.

*Verification:* T — test_state_machine_full_flight in test_apogee.c.

**PRG-FSW-055** The flight computer shall transition from `DESCENT` to
`LANDED` upon altitude remaining within ±2m and acceleration within
±0.5g of rest (1.0g) for not less than 10 seconds.

*Rationale:* Distinguishes a genuine landing from a brief lull in descent
(e.g. under parachute oscillation). Mirrors PRG-FSW-050's original wording.

*Verification:* T — test_state_machine_full_flight in test_apogee.c,
full ARMED->LANDED sequence.

**PRG-FSW-070** The flight computer shall accept start/stop/dump commands
over the serial interface, functionally equivalent to the physical arm
button, without requiring recompilation.

*Rationale:* Enables bench testing and automated test sequences without
physical access to the button; supports repeatable test procedures.

*Verification:* T — send each command over serial, confirm identical
behavior to the corresponding physical button action.

**PRG-FSW-080** Detection thresholds (boost g-force, burnout g-force,
landing stability window, hold durations) shall be adjustable without
firmware recompilation.

*Rationale:* Different motors and airframes require different thresholds;
hardcoded constants require a full rebuild-and-reflash cycle to tune,
which is impractical for iterative ground testing.

*Verification:* T — change a threshold value via config, confirm detector
behavior changes accordingly without rebuilding.

## Operations

**PRG-OPS-050** The flight computer shall provide on-demand retrieval of
recorded telemetry via a physical control, without requiring the flight
computer to be connected to a ground station or laptop during flight.

*Rationale:* Enables inspection of recorded data immediately after
recovery, before physical access to a computer is available.

*Verification:* D — long-press of the arm button triggers a full CSV
dump of the current session's log file over serial, confirmed on
hardware.

**PRG-OPS-060** The flight computer's data storage shall be physically
removable, permitting retrieval of recorded flight data using a standard
computer without custom software.

*Rationale:* Reduces dependency on the flight computer's own firmware
or a serial connection for data retrieval; a removed storage medium is
readable by any computer.

*Verification:* D — SD card removed and file contents confirmed
readable as a standard FAT-formatted volume.

## Data recording

**PRG-DAT-040** The flight computer shall not overwrite data from a
previous flight.

*Rationale:* Sequential file naming prevents loss of a prior flight when
the FC is powered on for a second time.

*Verification:* T — test_flight_init_auto_does_not_overwrite in
test_apogee.c (host, LittleFS). Re-confirmed on hardware against the
FatFS/SD backend: sequential arm/stop cycles produced flight_001.bin
and flight_002.bin without overwriting.

**PRG-SEN-060** The flight computer shall cross-validate acceleration
readings between the MPU6050 and ADXL375 where both sensors are within
their valid measurement range, and flag disagreement exceeding a defined
tolerance.

*Rationale:* Two independent accelerometers reduce the risk of a single
sensor fault going undetected; disagreement between them is itself a
useful health signal.

*Verification:* T — feed both sensors known matching and mismatched
values via the fake bus, confirm correct flagging in both cases.

## Flight environment

**PRG-ENV-010** The flight computer's physical assembly (perfboard,
mounting, wiring, battery) shall survive sustained acceleration
consistent with the motor class used, plus a safety margin, without
mechanical failure of solder joints, connectors, or mounting hardware.

*Rationale:* A flight computer that logs data correctly but physically
fails during boost produces no usable data at all.

*Verification:* A — analysis of expected peak acceleration for the
intended motor, compared against mounting/mechanical design margins. No
test performed yet; full verification requires an actual flight or a
ground-based shock/vibration test.

**PRG-ENV-020** The flight computer shall operate correctly across the
expected ambient temperature range at the launch site and altitude.

*Rationale:* Sensor accuracy and battery performance both vary with
temperature; unverified operation outside typical indoor/bench
conditions is a real risk for an outdoor launch.

*Verification:* A — not yet tested. Requires either a controlled
temperature test or accepting the risk on a first flight in mild weather.

**PRG-ENV-030** The flight computer's battery shall provide sufficient
capacity to power the system from ground arming through landing and a
reasonable post-landing retrieval window (target: 30 minutes total).

*Rationale:* A battery that dies mid-descent or before recovery loses
the whole flight's data and defeats the purpose of the LED/long-press
retrieval features.

*Verification:* D — bench test: time from arm to battery depletion under
realistic current draw (all three sensors + SD card writes active).

## Deferred / Future Work

The following are known future needs, not yet committed to a specific
design:

- **Hardware-in-the-Loop (HIL) testing** — a second microcontroller
  impersonating sensor hardware over real I2C, to test failure modes
  and timing behavior that cannot be safely or easily produced with
  real sensors. Not started.
- **RTOS migration** — moving from the current bare-metal main loop to
  a real-time OS (e.g. FreeRTOS) with prioritized tasks, to guarantee
  sensor sampling timing independent of slower operations like SD
  writes. Considered, not committed; current bare-metal design has not
  shown a timing problem that would require this yet.
- **Radio telemetry** — real-time downlink to a ground station during
  flight. Blocked on selecting and acquiring LoRa hardware.
- **First actual flight** — every requirement above is currently
  verified against bench tests and synthetic data. No requirement in
  this document has been verified against a real flight.


## Verification matrix
## Verification matrix

| Requirement | Method | Status |
|---|---|---|
| PRG-FSW-030 | T | **Met** — velocity detector using a 10-sample ring buffer, split into two groups of 5 for averaging: 140ms clean, 180ms with ±0.2m sensor noise. Raw two-sample velocity (superseded) measured 60ms clean but 600ms under the same noise — discarded for being noise-sensitive despite looking better on clean data. Distance-based detector (apogee.c) measures 500ms; retained as a baseline. |
| PRG-FSW-020 | T | **Met** — declared 100ms after sustained 5g onset (the theoretical minimum given the 100ms hold requirement); single-sample 5g spike correctly produces no detection. |
| PRG-FSW-025 | T | **Met** — burnout declared 100ms after sustained sub-1.5g onset on clean data, and under ±0.1g sensor noise (starting mid-boost, consistent with how it's actually consulted by the state machine). |
| PRG-FSW-065 | T | **Met** — full synthetic flight correctly reaches BOOST at 1100ms and DESCENT at 5140ms. |
| PRG-FSW-055 | T | **Met** — landed declared at the theoretical minimum (10000ms after ground phase begins) under simultaneous ±0.3m altitude noise and ±0.1g acceleration noise. |
| PRG-OPS-050 | D | **Met** — long-press dump confirmed on hardware, current session's file read back correctly as CSV. |
| PRG-OPS-060 | D | **Met** — SD card confirmed readable as a standard FAT volume after removal. |
| PRG-DAT-040 | T | **Met** — test_flight_init_auto_does_not_overwrite confirms a second flight lands on flight_002.bin and flight_001.bin's content remains intact and readable, on both LittleFS (host) and FatFS/SD (hardware). |