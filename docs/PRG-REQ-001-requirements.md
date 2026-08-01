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

## Verification matrix

| Requirement | Method | Status |
|---|---|---|
| PRG-FSW-030 | T | **Not met** — 500 ms measured, see test_apogee.c |
