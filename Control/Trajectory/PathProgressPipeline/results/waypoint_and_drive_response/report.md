# Waypoint stops and drive-response sensitivity

MATLAB 2024b. Drive parameters are illustrative simulation values.

The stop gate used 0.00035 rad position error, 0.003 rad/s actual joint speed, 0.05 s dwell and 1.25 s timeout. The 1 ms gate task conservatively certified the constant-input interval between events. Periodic commands stayed on the wall clock.

| Group | Case | Complete | Timeout | Provisional violations |
|---|---|---:|---:|---:|
| Matrix | Exact-length circle | 36 | 0 | 0 |
| Matrix | Multi S-curve | 24 | 12 | 0 |
| Offset subset | Exact-length circle | 4 | 0 | 0 |
| Offset subset | Multi S-curve | 2 | 2 | 0 |
| Stop comparison | Multi S-curve | 8 | 0 | 0 |
| Negative | Multi S-curve | 0 | 2 | 0 |

## 50 ms command examples

Values compare ideal and 50 ms lag plus 10 ms delay, in that order. All rows start matched. Timeout metrics cover only the executed prefix. Periodic circle completion marks the end of planned motion.

| Case | Gain (1/s) | Outcome | RMS TCP (mm) | Peak speed error (mm/s) | Added settling (s) |
|---|---:|---|---:|---:|---:|
| Exact-length circle | 2 | Complete → Complete | 0.835 → 2.969 | 11.46 → 19.83 | 0 → 0 |
| Exact-length circle | 8 | Complete → Complete | 0.257 → 0.9987 | 9.077 → 13.42 | 0 → 0 |
| Multi S-curve | 2 | Timeout → Timeout | 0.678 → 2.157 | 11.24 → 18.17 | 3.391 → 1.25 |
| Multi S-curve | 8 | Complete → Complete | 0.2723 → 1.071 | 8.956 → 14.23 | 0.7899 → 2.279 |

## Boundary comparison with the fixed offset at 50 ms

| Stop handling | Wall time (s) | Added settling (s) | Peak TCP (mm) | First boundary aligned | Second boundary aligned |
|---|---:|---:|---:|---|---|
| false | 21.3601 | 0 | 4.07107 | true | false |
| true | 22.15 | 0.789924 | 4.07107 | true | false |

Both negative tests timed out at the first waypoint and the next segment was never started. The delayed test used the same queue for boundary and periodic commands.

## Observation convergence

- run039, Exact-length circle. At 1 → 0.25 ms observation spacing, peak TCP error 6.71491 → 6.71492 mm and peak speed error 19.8332 → 19.8332 mm/s.
- run070, Multi S-curve. At 1 → 0.25 ms observation spacing, peak TCP error 5.68281 → 5.68281 mm and peak speed error 18.1687 → 18.1687 mm/s.
- run083, Multi S-curve. At 1 → 0.25 ms observation spacing, peak TCP error 4.07107 → 4.07107 mm and peak speed error 32.6049 → 32.6049 mm/s.

The plant starts at rest with zero held input. A boundary command uses the same delay as a periodic command. The simulation ends at the terminal event and reports any still-pending commands. Rate steps in the zero-lag model do not represent physical acceleration or jerk. Hardware work requires measured velocity-loop response, transport latency and jitter, limits, and a defined stop acceptance criterion.
