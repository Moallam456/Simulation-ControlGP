%% Compare Time-Scaling Methods
%
% PURPOSE
% -------
% This script compares:
%
%   1. Cubic polynomial time scaling
%   2. Quintic polynomial time scaling
%   3. Trapezoidal time scaling
%   4. S-curve jerk-limited time scaling
%
%
% IMPORTANT:
% ----------
% This script does NOT use:
%
%   - robot kinematics
%   - inverse kinematics
%   - rigidBodyTree
%   - animation
%
% Those are handled separately in testTimedLinearMotion.m.
%
%
% Here we want to answer:
%
%   "How do the TIME-SCALING METHODS themselves differ?"
%
%
% We perform two main experiments:
%
%   EXPERIMENT A:
%       Same total motion time
%
%       -> Compare shape and smoothness fairly.
%
%
%   EXPERIMENT B:
%       Same maximum path velocity and acceleration
%
%       -> Compare how quickly each method can complete the path
%          while respecting equivalent motion constraints.
%
%
% Finally:
%
%   EXPERIMENT C:
%       Jerk-aware comparison
%
%       -> Compare quintic and S-curve when jerk is also constrained.
%
%
% Throughout the script:
%
%       s = 0   -> beginning of path
%       s = 1   -> end of path
%
% and:
%
%       sDot     = normalized path velocity       [1/s]
%       sDDot    = normalized path acceleration   [1/s^2]
%       sDDDot   = normalized path jerk           [1/s^3]


clear
clc
close all


%% ========================================================================
%  1. COMMON TEST PARAMETERS
% =========================================================================

% Sampling period used for all trajectory generation.
%
% Smaller dt:
%   -> more samples
%   -> smoother plots
%   -> more computation
%
% This is NOT necessarily the final control-loop period.

dt = 0.01;                     % [s]


% Physical Cartesian line length.
%
% This lets us convert normalized quantities into meaningful TCP units.
%
% For our current straight-line test:
%
%       A -> B = 10 cm
%
% therefore:
%
%       L = 0.10 m

pathLength = 0.10;             % [m]


%% ========================================================================
%  2. COMMON MOTION CONSTRAINTS
% =========================================================================

% These are NORMALIZED path constraints.
%
% They are limits on:
%
%       sDot
%       sDDot
%       sDDDot
%
% They are NOT directly joint velocity/acceleration limits.


vMax = 0.4;                    % Maximum sDot   [1/s]

aMax = 0.4;                    % Maximum sDDot  [1/s^2]

jMax = 1.0;                    % Maximum sDDDot [1/s^3]


%% ========================================================================
%
%               EXPERIMENT A — SAME TOTAL MOTION TIME
%
% =========================================================================
%
% QUESTION:
%
%   If every profile has the SAME duration, how do their:
%
%       velocity
%       acceleration
%       jerk
%
%   characteristics differ?
%
%
% This isolates PROFILE SHAPE from total motion duration.
%
%
% We choose:
%
%       T = 4 seconds
%
% for every method.


Tcommon = 4.0;                 % [s]


fprintf("\n");
fprintf("============================================================\n");
fprintf("EXPERIMENT A - SAME TOTAL MOTION TIME\n");
fprintf("============================================================\n");
fprintf("Common duration: %.3f s\n",Tcommon);
fprintf("============================================================\n");


%% ========================================================================
%  A1. CUBIC — DIRECTLY GENERATE AT Tcommon
% =========================================================================

[tC_A, sC_A, sDotC_A, sDDotC_A, sDDDotC_A, infoC_A] = ...
    cubicTimeScaling( ...
        Tcommon, ...
        dt);


%% ========================================================================
%  A2. QUINTIC — DIRECTLY GENERATE AT Tcommon
% =========================================================================

[tQ_A, sQ_A, sDotQ_A, sDDotQ_A, sDDDotQ_A, infoQ_A] = ...
    quinticTimeScaling( ...
        Tcommon, ...
        dt);


%% ========================================================================
%  A3. TRAPEZOIDAL — GENERATE ITS NATURAL PROFILE FIRST
% =========================================================================

% Our trapezoidal function does NOT accept T directly.
%
% Instead we specify:
%
%       vMax
%       aMax
%
% and the function calculates its fastest duration.
%
% With our current parameters this duration is approximately:
%
%       3.5 s
%
%
% But Experiment A requires:
%
%       4.0 s
%
%
% Therefore we first generate the profile normally...

[tT_native, sT_native, sDotT_native, ...
    sDDotT_native, sDDDotT_native, infoT_native] = ...
    trapezoidalTimeScaling( ...
        vMax, ...
        aMax, ...
        dt);


%% ========================================================================
%  A4. STRETCH TRAPEZOIDAL PROFILE TO Tcommon
% =========================================================================

% We stretch its TIME axis.
%
% Example:
%
%       original duration = 3.5 s
%       desired duration  = 4.0 s
%
%
% Time-scale factor:
%
%       scale = 4 / 3.5
%
%
% Position progress s does NOT change.
%
% But derivatives change because the same motion now takes longer.
%
%
% If:
%
%       t_new = scale * t_old
%
% then:
%
%       sDot_new   = sDot_old   / scale
%
%       sDDot_new  = sDDot_old  / scale^2
%
%       sDDDot_new = sDDDot_old / scale^3


scaleT = Tcommon / infoT_native.T;


tT_A = tT_native * scaleT;

sT_A = sT_native;

sDotT_A = sDotT_native / scaleT;

sDDotT_A = sDDotT_native / scaleT^2;

sDDDotT_A = sDDDotT_native / scaleT^3;


%% ========================================================================
%  A5. S-CURVE — GENERATE NATURAL PROFILE
% =========================================================================

[tS_native, sS_native, sDotS_native, ...
    sDDotS_native, sDDDotS_native, infoS_native] = ...
    sCurveTimeScaling( ...
        vMax, ...
        aMax, ...
        jMax, ...
        dt);


%% ========================================================================
%  A6. STRETCH S-CURVE TO Tcommon
% =========================================================================

scaleS = Tcommon / infoS_native.T;


tS_A = tS_native * scaleS;

sS_A = sS_native;

sDotS_A = sDotS_native / scaleS;

sDDotS_A = sDDotS_native / scaleS^2;

sDDDotS_A = sDDDotS_native / scaleS^3;


%% ========================================================================
%  A7. CALCULATE PEAK VALUES
% =========================================================================

% ------------------------------------------------------------------------
% Cubic
% ------------------------------------------------------------------------

peakVelC_A = max(abs(sDotC_A));

peakAccC_A = max(abs(sDDotC_A));


% The cubic polynomial has finite jerk INSIDE the trajectory.
%
% However, acceleration at the start/end is non-zero.
%
% Connecting:
%
%       stationary robot
%
% to:
%
%       non-zero acceleration
%
% creates an instantaneous acceleration change.
%
% Therefore cubic is NOT truly jerk-bounded at its boundaries.

finiteJerkC_A = ...
    max(abs(sDDDotC_A(isfinite(sDDDotC_A))));


% ------------------------------------------------------------------------
% Quintic
% ------------------------------------------------------------------------

peakVelQ_A = max(abs(sDotQ_A));

peakAccQ_A = max(abs(sDDotQ_A));

peakJerkQ_A = max(abs(sDDDotQ_A));


% ------------------------------------------------------------------------
% Trapezoidal
% ------------------------------------------------------------------------

peakVelT_A = max(abs(sDotT_A));

peakAccT_A = max(abs(sDDotT_A));


% Trapezoidal acceleration jumps instantaneously at phase transitions.
%
% Therefore its ideal jerk at those transitions is unbounded.
%
% We do NOT report zero jerk simply because jerk is zero between switches.

finiteT = sDDDotT_A(isfinite(sDDDotT_A));

peakFiniteJerkT_A = max(abs(finiteT));


% ------------------------------------------------------------------------
% S-curve
% ------------------------------------------------------------------------

peakVelS_A = max(abs(sDotS_A));

peakAccS_A = max(abs(sDDotS_A));

peakJerkS_A = max(abs(sDDDotS_A));


%% ========================================================================
%  A8. CONVERT TO PHYSICAL TCP VALUES
% =========================================================================

% For a straight Cartesian path:
%
%       p(t) = pStart + deltaP*s(t)
%
% where:
%
%       ||deltaP|| = pathLength
%
%
% Therefore:
%
%       TCP speed        = pathLength * |sDot|
%
%       TCP acceleration = pathLength * |sDDot|
%
%       TCP jerk         = pathLength * |sDDDot|


tcpVelC_A = pathLength * peakVelC_A;
tcpVelQ_A = pathLength * peakVelQ_A;
tcpVelT_A = pathLength * peakVelT_A;
tcpVelS_A = pathLength * peakVelS_A;


tcpAccC_A = pathLength * peakAccC_A;
tcpAccQ_A = pathLength * peakAccQ_A;
tcpAccT_A = pathLength * peakAccT_A;
tcpAccS_A = pathLength * peakAccS_A;


%% ========================================================================
%  A9. DISPLAY SAME-DURATION SUMMARY TABLE
% =========================================================================

Method_A = [
    "Cubic"
    "Quintic"
    "Trapezoidal"
    "S-Curve"
];


Duration_A = [
    Tcommon
    Tcommon
    Tcommon
    Tcommon
];


Peak_sDot_A = [
    peakVelC_A
    peakVelQ_A
    peakVelT_A
    peakVelS_A
];


Peak_sDDot_A = [
    peakAccC_A
    peakAccQ_A
    peakAccT_A
    peakAccS_A
];


Peak_TCP_Speed_A = [
    tcpVelC_A
    tcpVelQ_A
    tcpVelT_A
    tcpVelS_A
];


Peak_TCP_Acceleration_A = [
    tcpAccC_A
    tcpAccQ_A
    tcpAccT_A
    tcpAccS_A
];


JerkBehavior_A = [
    "Boundary acceleration discontinuity"
    "Finite jerk"
    "Acceleration discontinuities"
    "Bounded jerk"
];


comparisonA = table( ...
    Method_A, ...
    Duration_A, ...
    Peak_sDot_A, ...
    Peak_sDDot_A, ...
    Peak_TCP_Speed_A, ...
    Peak_TCP_Acceleration_A, ...
    JerkBehavior_A);


fprintf("\nSAME-DURATION RESULTS\n\n");

disp(comparisonA);


%% ========================================================================
%  A10. PLOT ALL FOUR s(t)
% =========================================================================

figure

tiledlayout(4,1)


% ------------------------------------------------------------------------
% Path progress
% ------------------------------------------------------------------------

nexttile

plot(tC_A,sC_A,'LineWidth',1.5)
hold on

plot(tQ_A,sQ_A,'LineWidth',1.5)

plot(tT_A,sT_A,'LineWidth',1.5)

plot(tS_A,sS_A,'LineWidth',1.5)

grid on

ylabel('s')

title('Same Duration: Path Progress')

legend( ...
    'Cubic', ...
    'Quintic', ...
    'Trapezoidal', ...
    'S-Curve', ...
    'Location','best');


% ------------------------------------------------------------------------
% Path velocity
% ------------------------------------------------------------------------

nexttile

plot(tC_A,sDotC_A,'LineWidth',1.5)
hold on

plot(tQ_A,sDotQ_A,'LineWidth',1.5)

plot(tT_A,sDotT_A,'LineWidth',1.5)

plot(tS_A,sDotS_A,'LineWidth',1.5)

grid on

ylabel('ds/dt [1/s]')

title('Same Duration: Path Velocity');


% ------------------------------------------------------------------------
% Path acceleration
% ------------------------------------------------------------------------

nexttile

plot(tC_A,sDDotC_A,'LineWidth',1.5)
hold on

plot(tQ_A,sDDotQ_A,'LineWidth',1.5)

plot(tT_A,sDDotT_A,'LineWidth',1.5)

plot(tS_A,sDDotS_A,'LineWidth',1.5)

grid on

ylabel('d^2s/dt^2 [1/s^2]')

title('Same Duration: Path Acceleration');


% ------------------------------------------------------------------------
% Path jerk
% ------------------------------------------------------------------------

nexttile

plot(tC_A,sDDDotC_A,'LineWidth',1.5)
hold on

plot(tQ_A,sDDDotQ_A,'LineWidth',1.5)

plot(tT_A,sDDDotT_A,'LineWidth',1.5)

plot(tS_A,sDDDotS_A,'LineWidth',1.5)

grid on

ylabel('d^3s/dt^3 [1/s^3]')
xlabel('Time [s]')

title('Same Duration: Path Jerk');


%% ========================================================================
%
%       EXPERIMENT B — SAME VELOCITY + ACCELERATION CONSTRAINTS
%
% =========================================================================
%
% QUESTION:
%
%   If every method must obey:
%
%       |sDot|  <= vMax
%
%       |sDDot| <= aMax
%
%   how quickly can it complete the path?
%
%
% This experiment is closer to:
%
%   "How fast can this profile move under given limits?"
%
%
% NOTE:
%
% The S-curve also obeys jMax.
%
% Therefore it has an ADDITIONAL constraint compared with the other
% profiles.


fprintf("\n");
fprintf("============================================================\n");
fprintf("EXPERIMENT B - SAME VELOCITY / ACCELERATION LIMITS\n");
fprintf("============================================================\n");

fprintf("vMax = %.3f 1/s\n",vMax);
fprintf("aMax = %.3f 1/s^2\n",aMax);

fprintf("============================================================\n");


%% ========================================================================
%  B1. FIND MINIMUM CUBIC DURATION
% =========================================================================

% For cubic time scaling:
%
%       max |sDot| = 3/(2T)
%
% Therefore the velocity requirement is:
%
%       3/(2T) <= vMax
%
% giving:
%
%       T >= 3/(2*vMax)


T_C_velocity = ...
    3 / (2*vMax);


% Cubic peak acceleration:
%
%       max |sDDot| = 6/T^2
%
% Requirement:
%
%       6/T^2 <= aMax
%
% giving:
%
%       T >= sqrt(6/aMax)


T_C_acceleration = ...
    sqrt(6/aMax);


% Both constraints must be satisfied.
%
% Therefore choose the larger required duration.

T_C_B = max( ...
    T_C_velocity, ...
    T_C_acceleration);


% Generate cubic profile using this minimum feasible duration.

[tC_B, sC_B, sDotC_B, ...
    sDDotC_B, sDDDotC_B, infoC_B] = ...
    cubicTimeScaling( ...
        T_C_B, ...
        dt);


%% ========================================================================
%  B2. FIND MINIMUM QUINTIC DURATION
% =========================================================================

% Quintic peak velocity:
%
%       max |sDot| = 15/(8T)
%
%       = 1.875/T
%
% Therefore:

T_Q_velocity = ...
    1.875 / vMax;


% Quintic peak acceleration:
%
%       max |sDDot|
%
%           10
%       = --------
%         sqrt(3) T^2
%
%
% Therefore:

quinticAccelerationCoefficient = ...
    10 / sqrt(3);


T_Q_acceleration = ...
    sqrt( ...
        quinticAccelerationCoefficient / aMax);


% Again, both constraints must be satisfied.

T_Q_B = max( ...
    T_Q_velocity, ...
    T_Q_acceleration);


[tQ_B, sQ_B, sDotQ_B, ...
    sDDotQ_B, sDDDotQ_B, infoQ_B] = ...
    quinticTimeScaling( ...
        T_Q_B, ...
        dt);


%% ========================================================================
%  B3. TRAPEZOIDAL PROFILE
% =========================================================================

% Our trapezoidal function already directly solves:
%
%       vMax + aMax
%
%       ↓
%
%       fastest corresponding trajectory


[tT_B, sT_B, sDotT_B, ...
    sDDotT_B, sDDDotT_B, infoT_B] = ...
    trapezoidalTimeScaling( ...
        vMax, ...
        aMax, ...
        dt);


%% ========================================================================
%  B4. S-CURVE PROFILE
% =========================================================================

% S-curve directly receives:
%
%       vMax
%       aMax
%       jMax
%
%
% Therefore it respects the same velocity and acceleration limits,
% PLUS an additional jerk constraint.

[tS_B, sS_B, sDotS_B, ...
    sDDotS_B, sDDDotS_B, infoS_B] = ...
    sCurveTimeScaling( ...
        vMax, ...
        aMax, ...
        jMax, ...
        dt);


%% ========================================================================
%  B5. VERIFY CONSTRAINTS
% =========================================================================

% These asserts act as automatic sanity checks.
%
% If any profile exceeds the intended limit, MATLAB stops the script.


assert( ...
    max(abs(sDotC_B)) <= vMax + 1e-9, ...
    "Cubic exceeds vMax.");


assert( ...
    max(abs(sDDotC_B)) <= aMax + 1e-9, ...
    "Cubic exceeds aMax.");


assert( ...
    max(abs(sDotQ_B)) <= vMax + 1e-9, ...
    "Quintic exceeds vMax.");


assert( ...
    max(abs(sDDotQ_B)) <= aMax + 1e-9, ...
    "Quintic exceeds aMax.");


assert( ...
    max(abs(sDotT_B)) <= vMax + 1e-9, ...
    "Trapezoidal exceeds vMax.");


assert( ...
    max(abs(sDDotT_B)) <= aMax + 1e-9, ...
    "Trapezoidal exceeds aMax.");


assert( ...
    max(abs(sDotS_B)) <= vMax + 1e-9, ...
    "S-curve exceeds vMax.");


assert( ...
    max(abs(sDDotS_B)) <= aMax + 1e-9, ...
    "S-curve exceeds aMax.");


%% ========================================================================
%  B6. CALCULATE ACTUAL PEAK VALUES
% =========================================================================

peakVelC_B = max(abs(sDotC_B));
peakVelQ_B = max(abs(sDotQ_B));
peakVelT_B = max(abs(sDotT_B));
peakVelS_B = max(abs(sDotS_B));


peakAccC_B = max(abs(sDDotC_B));
peakAccQ_B = max(abs(sDDotQ_B));
peakAccT_B = max(abs(sDDotT_B));
peakAccS_B = max(abs(sDDotS_B));


%% ========================================================================
%  B7. CREATE CONSTRAINT-COMPARISON TABLE
% =========================================================================

Method_B = [
    "Cubic"
    "Quintic"
    "Trapezoidal"
    "S-Curve"
];


Duration_B = [
    infoC_B.T
    infoQ_B.T
    infoT_B.T
    infoS_B.T
];


Peak_sDot_B = [
    peakVelC_B
    peakVelQ_B
    peakVelT_B
    peakVelS_B
];


Peak_sDDot_B = [
    peakAccC_B
    peakAccQ_B
    peakAccT_B
    peakAccS_B
];


Peak_TCP_Speed_B = ...
    pathLength * Peak_sDot_B;


Peak_TCP_Acceleration_B = ...
    pathLength * Peak_sDDot_B;


JerkTreatment_B = [
    "Not bounded at motion boundaries"
    "Finite but not constrained here"
    "Not bounded at acceleration switches"
    "Explicitly limited"
];


comparisonB = table( ...
    Method_B, ...
    Duration_B, ...
    Peak_sDot_B, ...
    Peak_sDDot_B, ...
    Peak_TCP_Speed_B, ...
    Peak_TCP_Acceleration_B, ...
    JerkTreatment_B);


fprintf("\nVELOCITY / ACCELERATION CONSTRAINED RESULTS\n\n");

disp(comparisonB);


%% ========================================================================
%  B8. PLOT VELOCITIES UNDER SAME CONSTRAINTS
% =========================================================================

figure

tiledlayout(2,1)


nexttile

plot(tC_B,sDotC_B,'LineWidth',1.5)
hold on

plot(tQ_B,sDotQ_B,'LineWidth',1.5)

plot(tT_B,sDotT_B,'LineWidth',1.5)

plot(tS_B,sDotS_B,'LineWidth',1.5)


% Plot velocity limit for visual reference.

yline(vMax,'--');


grid on

ylabel('ds/dt [1/s]')

title('Same Constraints: Path Velocity')

legend( ...
    'Cubic', ...
    'Quintic', ...
    'Trapezoidal', ...
    'S-Curve', ...
    'vMax', ...
    'Location','best');


nexttile

plot(tC_B,sDDotC_B,'LineWidth',1.5)
hold on

plot(tQ_B,sDDotQ_B,'LineWidth',1.5)

plot(tT_B,sDDotT_B,'LineWidth',1.5)

plot(tS_B,sDDotS_B,'LineWidth',1.5)


% Positive and negative acceleration limits.

yline(aMax,'--');

yline(-aMax,'--');


grid on

ylabel('d^2s/dt^2 [1/s^2]')
xlabel('Time [s]')

title('Same Constraints: Path Acceleration');


%% ========================================================================
%  B9. COMPARE TRAJECTORY DURATIONS
% =========================================================================

figure


bar( ...
    categorical(Method_B), ...
    Duration_B);


grid on

ylabel('Trajectory duration [s]')

title('Motion Time Under Common Velocity / Acceleration Limits');


%% ========================================================================
%
%               EXPERIMENT C — JERK-AWARE COMPARISON
%
% =========================================================================
%
% Cubic and trapezoidal profiles have acceleration discontinuities.
%
% Therefore their ideal jerk becomes unbounded at those discontinuities.
%
%
% It is therefore misleading to compare them against a strict finite
% jerk limit.
%
%
% Quintic and S-curve, however, can meaningfully be compared under:
%
%       velocity limit
%       acceleration limit
%       jerk limit


fprintf("\n");
fprintf("============================================================\n");
fprintf("EXPERIMENT C - JERK-AWARE COMPARISON\n");
fprintf("============================================================\n");


%% ========================================================================
%  C1. QUINTIC JERK REQUIREMENT
% =========================================================================

% Quintic maximum jerk is:
%
%       max |sDDDot| = 60/T^3
%
%
% To satisfy:
%
%       max |sDDDot| <= jMax
%
% we require:
%
%       T >= (60/jMax)^(1/3)


T_Q_jerk = ...
    (60/jMax)^(1/3);


% To satisfy velocity, acceleration AND jerk:
%
% choose the largest of all three required durations.

T_Q_C = max([
    T_Q_velocity
    T_Q_acceleration
    T_Q_jerk
]);


[tQ_C, sQ_C, sDotQ_C, ...
    sDDotQ_C, sDDDotQ_C, infoQ_C] = ...
    quinticTimeScaling( ...
        T_Q_C, ...
        dt);


%% ========================================================================
%  C2. S-CURVE
% =========================================================================

% S-curve already directly receives all three constraints.

[tS_C, sS_C, sDotS_C, ...
    sDDotS_C, sDDDotS_C, infoS_C] = ...
    sCurveTimeScaling( ...
        vMax, ...
        aMax, ...
        jMax, ...
        dt);


%% ========================================================================
%  C3. VERIFY JERK LIMIT
% =========================================================================

assert( ...
    max(abs(sDDDotQ_C)) <= jMax + 1e-9, ...
    "Quintic exceeds jerk limit.");


assert( ...
    max(abs(sDDDotS_C)) <= jMax + 1e-9, ...
    "S-curve exceeds jerk limit.");


%% ========================================================================
%  C4. DISPLAY JERK-AWARE RESULTS
% =========================================================================

Method_C = [
    "Quintic"
    "S-Curve"
];


Duration_C = [
    infoQ_C.T
    infoS_C.T
];


PeakVelocity_C = [
    max(abs(sDotQ_C))
    max(abs(sDotS_C))
];


PeakAcceleration_C = [
    max(abs(sDDotQ_C))
    max(abs(sDDotS_C))
];


PeakJerk_C = [
    max(abs(sDDDotQ_C))
    max(abs(sDDDotS_C))
];


comparisonC = table( ...
    Method_C, ...
    Duration_C, ...
    PeakVelocity_C, ...
    PeakAcceleration_C, ...
    PeakJerk_C);


fprintf("\nVELOCITY + ACCELERATION + JERK RESULTS\n\n");

disp(comparisonC);


%% ========================================================================
%  C5. PLOT QUINTIC VS S-CURVE JERK
% =========================================================================

figure


plot( ...
    tQ_C, ...
    sDDDotQ_C, ...
    'LineWidth',1.5);

hold on


plot( ...
    tS_C, ...
    sDDDotS_C, ...
    'LineWidth',1.5);


yline(jMax,'--');

yline(-jMax,'--');


grid on

xlabel('Time [s]')

ylabel('d^3s/dt^3 [1/s^3]')

title('Jerk-Limited Comparison');


legend( ...
    'Quintic', ...
    'S-Curve', ...
    '+jMax', ...
    '-jMax', ...
    'Location','best');


%% ========================================================================
%  FINAL SUMMARY
% =========================================================================

fprintf("\n");
fprintf("============================================================\n");
fprintf("FINAL INTERPRETATION GUIDE\n");
fprintf("============================================================\n");

fprintf("\nExperiment A:\n");
fprintf("Compare motion SHAPE when duration is held constant.\n");

fprintf("\nExperiment B:\n");
fprintf("Compare required motion TIME under the same velocity and acceleration limits.\n");

fprintf("\nExperiment C:\n");
fprintf("Compare Quintic vs S-Curve when jerk is also explicitly limited.\n");

fprintf("\nDo NOT select the final method from trajectory time alone.\n");
fprintf("The next step is to observe the resulting TCP and joint motion.\n");

fprintf("============================================================\n");