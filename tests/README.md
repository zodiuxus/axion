# Tests

The firmware has no on-target test harness (no Unity component, no board in the
loop). These tests cover the **pure logic** that decides whether the device
alarms, which is where the safety-relevant rules live. They compile for the
host, so they run in under a second and can be wired into CI.

## What is covered

| Module | Why it is testable |
|---|---|
| `main/alert_policy.cpp` | Depends only on `config_tuning.h` (no ESP-IDF types, no FreeRTOS, no logging). |
| `main/alert_message.cpp` | Formats the SMS body from plain values; no state, no ESP-IDF. |
| `main/collision.cpp` | Same idea — pure magnitude/cooldown predicate (no test yet). |

The ESP-IDF-dependent shell (tasks, ISRs, UART, I2C, NVS, the delivery worker)
is deliberately not covered here; it needs hardware and is validated by the
build plus a bench run.

## Running

Windows (MSVC, no ESP-IDF needed):

```powershell
powershell -File tests/host/run.ps1
```

Linux / macOS (any C++17 compiler):

```sh
g++ -std=c++17 -I main \
    main/alert_policy.cpp main/alert_message.cpp \
    tests/host/*.cpp \
    -o /tmp/axion_host_tests
/tmp/axion_host_tests
```

Both print the number of checks and return non-zero on failure.

## Adding a test

1. Keep the module free of ESP-IDF includes; include `config_tuning.h` for
   thresholds instead of `config.h`.
2. Create `tests/host/<area>_test.cpp`, include `test_util.h`, put the cases in
   an anonymous namespace and export `void run_<area>_tests()`.
3. Call it from `tests/host/run_all.cpp`; `run.ps1` picks the file up
   automatically.

## Temperature policy tests

The temperature band is measured from the calibrated baseline only:
`test_temperature_low_raised_relative_to_baseline`,
`test_temperature_band_follows_baseline` and
`test_temperature_off_body_reads_as_hypothermia` pin the behaviour. The last
one is the documented trade-off: an off-body probe reads ambient and is
reported as hypothermia. If that proves too noisy in the field, add a contact
signal or a longer debounce — do not reintroduce an absolute floor, which is
what hid real hypothermia.
