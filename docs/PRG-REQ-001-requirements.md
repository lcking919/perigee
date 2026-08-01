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

## Verification matrix

| Requirement | Method | Status |
|---|---|---|
| PRG-FSW-030 | T | **Met** — velocity detector using a 10-sample ring buffer, split into two groups of 5 for averaging: 140ms clean, 180ms with ±0.2m sensor noise. Raw two-sample velocity (superseded) measured 60ms clean but 600ms under the same noise — discarded for being noise-sensitive despite looking better on clean data. Distance-based detector (apogee.c) measures 500ms; retained as a baseline. |
| PRG-FSW-020 | T | **Met** — declared 100ms after sustained 5g onset (the theoretical minimum given the 100ms hold requirement); single-sample 5g spike correctly produces no detection. |
| PRG-FSW-025 | T | **Met** — burnout declared 100ms after sustained sub-1.5g onset (theoretical minimum given the hold requirement), verified as part of the full-flight state machine test. |
| PRG-FSW-065 | T | **Met** — full synthetic flight correctly reaches BOOST at 1100ms and DESCENT at 5140ms. |