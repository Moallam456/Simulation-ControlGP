# Ideal velocity-command execution

MATLAB 2024b. Post-trajectory observation 1 s. Initial offset [deg] [0.2 -0.15 0.1 -0.2 0.12 -0.1].

## Exact-length timing cleanup

- arc_exact_length used 118 → 118 samples. Duration 5.849862703 → 5.849895354 s and peak intended speed 0.100000628 → 0.100000000 m/s.
- fullcircle_exact_length used 203 → 203 samples. Duration 10.074680580 → 10.074777961 s and peak intended speed 0.100001033 → 0.100000000 m/s.

## Tracking results

Each row selects the lowest motion RMS TCP position error in its group. Completion and post-observation errors are separate.

| Case | Start | Period (ms) | Kp (1/s) | RMS TCP (mm) | Peak TCP (mm) | Final TCP (mm) | Observed TCP (mm) |
|---|---|---:|---:|---:|---:|---:|---:|
| line | Matched | 1 | 8 | 0.0046596 | 0.013784 | 0.0064102 | 2.0823e-06 |
| line | Offset | 20 | 8 | 0.45058 | 4.4527 | 0.12594 | 2.0615e-05 |
| arc_exact_length | Matched | 1 | 8 | 0.0058543 | 0.014022 | 0.0064708 | 2.102e-06 |
| arc_exact_length | Offset | 10 | 8 | 0.4859 | 4.7419 | 0.064178 | 1.5352e-05 |
| fullcircle_exact_length | Matched | 1 | 8 | 0.0053976 | 0.013999 | 0.0064012 | 2.0794e-06 |
| fullcircle_exact_length | Offset | 5 | 8 | 0.39615 | 5.0486 | 0.031902 | 9.0796e-06 |
| multi_scurve | Matched | 1 | 8 | 0.0057588 | 0.014074 | 0.0064941 | 2.1095e-06 |
| multi_scurve | Offset | 5 | 8 | 0.2187 | 4.0711 | 0.032352 | 9.2075e-06 |

Matched starts show the sampling trend most directly. Across the four paths, feedforward RMS TCP error ranged from 2.252–2.398 mm at 50 ms and 0.04504–0.04796 mm at 1 ms.
At 1 ms with offset starts, feedforward RMS TCP error ranged from 4.615–5.295 mm and Kp=8/s reduced it to 0.2195–0.4893 mm. The feedback correction can raise peak speed error at the start.
The largest command magnitude was 0.588791 rad/s, and the largest adjacent command increment was 0.102616 rad/s.

0 cases flagged position violations and 0 flagged velocity violations against the model’s provisional limits.

At 30 of 144 internal waypoint observations a periodic tick coincided with the waypoint. Maximum held TCP speed at a waypoint was 0.0029587 m/s before and 0.0029587 m/s after.

## Observation-grid convergence

- fullcircle_exact_length, 1 ms, Kp=8/s, Offset. Peak TCP 5.04863 → 5.04863 mm and peak speed error 0.0404268 → 0.0404268 m/s from 0.5 to 0.125 ms.
- fullcircle_exact_length, 50 ms, Kp=0/s, Offset. Peak TCP 7.6866 → 7.6866 mm and peak speed error 0.0129079 → 0.0129079 m/s from 0.5 to 0.125 ms.

## Interpretation

Motion errors compare equal-time FK of simulated and reference joints. Cross-path error is distance to the finite requested geometry. Speed uses the Jacobian at simulated position and the held command. Command changes are velocity increments in an instantaneous kinematic model. Application acceptance limits and hardware acceleration and jerk ratings remain unspecified. Before choosing hardware rates or gains, obtain drive command semantics, command and feedback timing, and measured velocity-loop response.
