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

## Verification matrix

| Requirement | Method | Status |
|---|---|---|
| PRG-FSW-030 | T | **Met** — velocity detector using a 10-sample ring buffer, split into two groups of 5 for averaging: 140ms clean, 180ms with ±0.2m sensor noise. Raw two-sample velocity (superseded) measured 60ms clean but 600ms under the same noise — discarded for being noise-sensitive despite looking better on clean data. Distance-based detector (apogee.c) measures 500ms; retained as a baseline. |
| PRG-FSW-020 | T | **Met** — declared 100ms after sustained 5g onset (the theoretical minimum given the 100ms hold requirement); single-sample 5g spike correctly produces no detection. |