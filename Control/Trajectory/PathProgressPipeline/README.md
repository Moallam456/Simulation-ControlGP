# Path-progress trajectory demos

The four original demos and the S-curve variant expose the original
`qPath`, `qDot`, `qDDot` and timed Cartesian workspace arrays. They share the original
robot_ver2 loader, branch-selecting IK adapter, Cartesian waypoint generators,
arc-length resampling, SLERP, and time-scaling helpers. The arc uses a local
rotation-matrix adapter for the original script-local composition of path
rotation and leftover twist.

- `helpers/` contains the isolated planner, cubic core, validation, export,
  and visualization adapters.
- `tests/` captures the unchanged baseline and verifies the new entry scripts.
- `results/` contains the baseline and new MAT snapshots, a summary CSV, the
  line export with its original column schema, and four verification plots.

## Reproduce in MATLAB R2024b

Start MATLAB in the repository root. The saved baseline is ready for verification.
Regenerate it only when you intend to recapture the current original pipeline.
Use a fresh MATLAB session for capture because the original scripts close figures.
Capture runs disposable copies inside `results/` and records every original
multi-segment timing array before the original loop overwrites it.

```matlab
addpath(fullfile(pwd,'Control','Trajectory','PathProgressPipeline','tests'));
% capturePathProgressBaseline();  % Optional. Replaces the isolated baseline MAT.
verifyPathProgressPipeline();
```

To run a demo interactively from the repository root:

```matlab
run(fullfile(pwd,'Control','Trajectory','PathProgressPipeline','PathProgressDemo_arc.m'));
```

## Spatial path and timing

Progress is `s = l/L` for each segment, with IK nodes initially spaced at most
2.5 mm apart. These are spatial samples. The execution samples use the original
timestamps and normalized timing arrays. Every fit is checked at its knots, seven
interior points per interval, the original polyline corners, and execution
queries. Failed intervals are bisected locally, with up to six refinement
passes and 5,000 nodes per segment. Position-limit checks also cover each
cubic's entire Bézier control-point hull.

Each segment has a `1-by-N` knot array, a `joints-by-N` node array, and
`joints-by-(N-1)-by-4` coefficients ordered `[a,b,c,d]`. For
`u = s - nodes(k)`, the interval evaluates as:

```text
q    = a + b*u + c*u^2 + d*u^3
q_s  = b + 2*c*u + 3*d*u^2
q_ss = 2*c + 6*d*u
q_d   = q
qd_d  = q_s * s_dot
qdd_d = q_ss * s_dot^2 + q_s * s_ddot
```

Angles are in radians, time in seconds, and spatial derivatives are per unit
of dimensionless progress. The not-a-knot boundary conditions make the first
two and last two intervals share their third derivatives. This reproduces a
global cubic exactly and avoids prescribing artificial endpoint curvature
from the time-profile stops. An explicitly checked Thomas solve constructs
the coefficients. Internal queries use the right interval, and `s=1` uses
the last interval. Queries outside `[0,1]` raise an error.

Each segment hands its terminal joint configuration to the next segment and
owns its terminal timestamp. The single-segment S-curves have zero endpoint
velocity and acceleration. The original multi-segment trapezoids have zero
waypoint velocity and retain their one-sided endpoint acceleration. Joint jerk
can jump at spline knots and is undefined at ideal trapezoidal acceleration
switches. The results report sampled rate peaks and the provisional URDF
velocity comparison. The model provides no joint acceleration or jerk ratings.

## Verification and portability

The verifier compares original and new Cartesian poses and time scaling,
checks FK and joint continuity, and compares base-frame Jacobian twists with
the requested Cartesian derivatives. It checks knot continuity and uses
centered time perturbations of `1e-5 s` within individual timing phases.
FK limits are `1e-5 m` and `1e-4 rad`, the joint-step limit is `0.10 rad`,
and the endpoint branch tolerance is `1e-4 rad`. Planning comparisons use
`1e-12` in the corresponding units, and the timing arrays must match exactly.
Velocity and acceleration finite-difference tolerances are `2e-6 rad/s` and
`2e-4 rad/s^2`. The centered position difference has truncation error
bounded by `h^2 * max(abs(q_jerk))/6` within a timing phase. The centered
velocity difference has a larger allowance because crossing a spatial knot
can give first-order error when jerk jumps. Knot tolerances are
`1e-11 rad`, `1e-10 rad/progress`,
and `1e-8 rad/progress^2`. TCP twist tolerances are `2e-4 rad/s` angular
and `1e-4 m/s` linear plus the maximum adjacent-chord velocity jump. The
latter accounts for the tangent discontinuities in the unchanged polyline.

A C port can translate the cubic construction, Horner evaluation, and
chain rule directly with double-precision arrays, a preallocated workspace,
and the same validity checks. The existing Final_Architecture joint derivative
helper uses MATLAB spline and piecewise-polynomial APIs, so the new cubic core
implements the coefficient solve and evaluator explicitly. The geometry and timing generation,
robot loader, analytical IK, FK and Jacobian, URDF parsing, and graphics
remain MATLAB dependencies. A later C integration can consume precomputed
coefficient and timing arrays or supply equivalents for those planning and
robot-model dependencies.

## Twist and jerk follow-up

`PathProgressDemo_multisegment_scurve.m` uses the same five waypoint segments
and the existing S-curve generator. Each waypoint has zero reference velocity
and acceleration. The trapezoidal demo and its saved evidence remain available.
Run the follow-up against the existing `results/path_progress_after.mat`:

```matlab
addpath(fullfile(pwd,'Control','Trajectory','PathProgressPipeline','tests'));
analyzePathProgressFollowup();
```

- `path_progress_twist_diagnostics.csv` records the worst linear and angular
  samples for each demo. Linear error is resolved along and perpendicular to
  the selected Cartesian chord. The checks compare Jacobian predictions with
  finite differences of fitted-joint URDF FK and with direct IK/FK probes
  inside that chord.
- `path_progress_fullcircle_jerk_peaks.csv` compares the execution-sampled peak
  with a continuous search over timing phases and polynomial intervals.
  `path_progress_fullcircle_jerk_knots.csv` records both one-sided jerk values
  for every joint and interior knot.
- `path_progress_fullcircle_fk_convergence.csv` gives the errors on nested,
  independent uniform grids and the maxima after including exact Cartesian
  corners and joint knots. The diagnostic preserves the existing FK limits.
- `path_progress_multi_scurve.mat`, `path_progress_multi_scurve_stops.csv`, and
  `path_progress_multi_scurve_summary.csv` store the variant's references,
  waypoint stop results, and aggregate checks.

## Exact circular geometry comparison

`PathProgressDemo_arc_exact.m` and `PathProgressDemo_fullcircle_exact.m`
reuse the original circle metadata and orientation laws. The exact position
and its first three progress derivatives come from the signed circular sweep.
The original chord-length timing basis and all timing samples are preserved.
The exact variant reports actual distance using `radius*abs(sweep)` and
desired linear velocity using `p_s*s_dot`. The comparison CSV distinguishes
the raw chord length used for timing from the resampled polyline's travel
length and the exact circular length.

Run from the repository root in MATLAB R2024b:

```matlab
addpath(fullfile(pwd,'Control','Trajectory','PathProgressPipeline','tests'));
comparePathProgressExactGeometry();
```

The verifier reads the original chord references from
`results/path_progress_after.mat`. All new evidence goes to
`results/exact_geometry_comparison/`. The summary and MAT file retain
analytic and reconstructed speeds, FK errors, rate peaks, and direct IK
probes. The dense FK CSV includes up to 65,537 independent grid points
plus polynomial knots and the original chord-corner progress values.
The jerk CSV includes one-sided values at every interior joint knot and
at timing phase boundaries. The continuous peak search covers both sides of
each boundary and stationary points within each smooth time interval.
The cubic joint path remains C2, so jerk can still jump at joint knots or
timing phase boundaries. Hardware jerk limits remain unknown.

## Ideal velocity-command execution

`PathProgressDemo_arc_exact_length.m` and
`PathProgressDemo_fullcircle_exact_length.m` use exact circular length to
normalize the existing S-curve settings. The earlier chord-timed exact
variants remain available. The sweep also uses the saved line and multi-segment
S-curve references. The retiming CSV isolates changes in duration, samples and
peak intended TCP speed.

```matlab
addpath(fullfile(pwd,'Control','Trajectory','PathProgressPipeline','tests'));
verifyPathProgressVelocityExecution(1);  % observation seconds after completion
```

The numeric evaluator integrates the seven constant-jerk timing phases and
evaluates the existing joint cubic at progress. It returns desired joint
position, velocity and acceleration in radians and seconds. A held command
advances the simulated joints by its velocity times elapsed time. Commands
are periodic during motion with a mandatory update at exact completion.
The final pre-completion interval may be shorter. Post-completion ticks are
anchored at completion, and the final observation can fall inside a hold.
The executor uses the final position as the post-completion target and
zero reference velocity. Internal waypoints add observations only.

The sweep uses periods of 1, 2, 5, 10, 20 and 50 ms, with uniform joint
gains of 0, 2 or 8 per second. At a stationary target the ideal error multiplier
is `1 - Kp*dt`. The positive gains cover slower and faster monotone correction
with `Kp*dt <= 0.4`. These tests use matched starts and one fixed joint
offset of `[0.20 -0.15 0.10 -0.20 0.12 -0.10]` degrees. The state remains
continuous at command changes, and the command is never clipped.

Motion errors compare equal-time FK of reference and simulated joints.
Joint peaks use the largest absolute component, and joint RMS combines joints
and time. TCP speed uses the linear Jacobian at the simulated joints times
the held command. Time-weighted RMS accounts for left and right speed limits
at command ticks. Cross-path is the minimum 3-D distance to the finite
requested geometry, including arc endpoints and the union of finite straight
segments. The 0.5 ms observation grid also contains command, waypoint and
timing-phase events. Representative cases are repeated at 0.25 and 0.125 ms.
Rate and position flags use published provisional model values.

Results go to `results/velocity_execution/`. The executor and evaluator use
numeric arrays and elementary operations. The verification uses R2024b,
Robotics System Toolbox for independent URDF FK and Jacobian checks, and
MATLAB plotting and file I/O. Batch FK and Jacobian evaluation is derived
from the same URDF frames and checked against the toolbox. This model assumes
instantaneous velocity response. A command increment has units of rad/s and
does not establish a physical acceleration or jerk. Hardware suitability and
application tracking limits require separate drive information.

## Waypoint stops and delayed drive response

```matlab
addpath(fullfile(pwd,'Control','Trajectory','PathProgressPipeline','tests'));
verifyPathProgressWaypointDrive();
```

This is a separate experiment. It reads the exact-length circle and
multi-segment S-curve references from the saved velocity-execution MAT file.
It writes only to `results/waypoint_and_drive_response/`.

`pathProgressRunDrive` takes a packed numeric reference, initial joint
positions, a configuration, and provisional position/velocity limits.
Set `stops` to select boundary-aware or periodic execution. Position starts
at the supplied configuration. Actual velocity and held drive input start
at zero. Each command arrives at its transmit time plus the configured delay,
in transmit order, including boundary commands. The wall-clock periodic
ticks continue throughout settling and after a release. Coincident boundary
and periodic updates produce one command. Queues are retained across
boundaries. At the terminal event, the run stops and reports pending commands.

With stop handling, the reference clock freezes at an endpoint. Its desired
velocity is zero and its joint-position target stays active. The next segment
starts at local time zero when every joint meets the position and actual
velocity gate for the full dwell. The default simulation gate is 0.00035 rad,
0.003 rad/s, 0.05 s dwell, 1.25 s timeout, and 1 ms gate checks. These are
simulation parameters. The interval check conservatively verifies position
extrema and velocity endpoints between events. A timeout stops the run before
the next segment and remains distinct from normal completion.

The exact drive update uses held-input intervals split at command arrivals,
positive per-joint time constants in seconds, and instantaneous velocity
following at zero time constant. The portable core uses arrays, arithmetic,
and elementary functions, including a small-interval expansion for position.
The analytic observer uses the same update and reports pre-event velocity
where zero lag permits a jump. Verification measures equal-reference-time
tracking with the saved URDF-derived batch FK and Jacobian. The observation
grid includes exact events and is refined separately from the gate clock.
No transport or motor parameters in the sweep are hardware measurements.

## Verified single-segment endpoints

The existing drive configuration selects periodic completion with `stops=false`
or verified stop handling with `stops=true`. For the line, exact-length arc,
and exact-length full circle, the latter has one gate at the final planned
endpoint. The multi-segment variant continues to gate every waypoint. All
four use the same queue, drive update, and continuous-interval gate.

Run the endpoint comparison and focused multi-segment replay from the
repository root in MATLAB R2024b:

```matlab
addpath(fullfile(pwd,'Control','Trajectory','PathProgressPipeline','tests'));
verifyPathProgressEndpointSettling();
```

The optional input is a struct with any of `positionTol`,
`velocityTol`, `dwell`, `timeout`, and `gateStep`.
Defaults are the prior simulation values of 0.00035 rad, 0.003 rad/s,
50 ms dwell, 1.25 s timeout, and 1 ms monitor step. The test compares three
paired drive configurations, checks a deliberately failing gate, and
replays the saved multi-segment outcomes. The final dwell is independently
checked at 0.25 ms and over the constant-input intervals between events.
New results go to `results/endpoint_settling/`. The summary reports
instantaneous errors before and after the endpoint command and terminal
errors against the final planned target. An early multi-segment timeout
also records the local gate errors at the waypoint it actually reached.
