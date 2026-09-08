%% ========================================================================
%  TEST TIME-SCALING METHODS
%
% This script validates the mathematical behavior of:
%
%   1. Cubic polynomial time scaling
%   2. Quintic polynomial time scaling
%   3. Trapezoidal / triangular time scaling
%   4. S-curve jerk-limited time scaling
%
% No robot model or inverse kinematics is used here.
%
% The purpose is to verify:
%
%   - boundary conditions
%   - monotonic path progress
%   - velocity behavior
%   - acceleration behavior
%   - jerk behavior
%   - profile-specific constraints
%
% The robot integration test will be performed separately.


clear
clc
close all


%% ========================================================================
%  1. CUBIC TIME SCALING
% =========================================================================

% User parameters.
T_cubic  = 4.0;      % Total motion time [s]
dt_cubic = 0.01;     % Sampling period [s]


% Generate profile.
[tC, sC, sDotC, sDDotC, sDDDotC, infoC] = ...
    cubicTimeScaling(T_cubic, dt_cubic);


% ------------------------------------------------------------------------
% Boundary-condition checks
% ------------------------------------------------------------------------

fprintf("\n========================================\n");
fprintf("CUBIC TIME SCALING\n");
fprintf("========================================\n");

fprintf("s(0):        %.6f\n", sC(1));
fprintf("s(T):        %.6f\n", sC(end));

fprintf("sDot(0):     %.6f\n", sDotC(1));
fprintf("sDot(T):     %.6f\n", sDotC(end));

fprintf("sDDot(0):    %.6f\n", sDDotC(1));
fprintf("sDDot(T):    %.6f\n", sDDotC(end));

fprintf("Peak |sDot|:  %.6f 1/s\n", max(abs(sDotC)));
fprintf("Peak |sDDot|: %.6f 1/s^2\n", max(abs(sDDotC)));


% ------------------------------------------------------------------------
% General sanity checks
% ------------------------------------------------------------------------

% s must remain inside the normalized path interval.
assert(all(sC >= -1e-12 & sC <= 1 + 1e-12), ...
    "Cubic s leaves the interval [0,1].");

% s should never move backward.
assert(all(diff(sC) >= -1e-12), ...
    "Cubic profile is not monotonic.");


% ------------------------------------------------------------------------
% Plot cubic profile
% ------------------------------------------------------------------------

figure

tiledlayout(4,1)

nexttile
plot(tC,sC,'LineWidth',1.5)
grid on
ylabel('s')
title('Cubic Time Scaling')

nexttile
plot(tC,sDotC,'LineWidth',1.5)
grid on
ylabel('ds/dt [1/s]')

nexttile
plot(tC,sDDotC,'LineWidth',1.5)
grid on
ylabel('d^2s/dt^2 [1/s^2]')

nexttile
plot(tC,sDDDotC,'LineWidth',1.5)
grid on
ylabel('d^3s/dt^3 [1/s^3]')
xlabel('Time [s]')


%% ========================================================================
%  2. QUINTIC TIME SCALING
% =========================================================================

T_quintic  = 4.0;
dt_quintic = 0.01;


[tQ, sQ, sDotQ, sDDotQ, sDDDotQ, infoQ] = ...
    quinticTimeScaling(T_quintic, dt_quintic);


fprintf("\n========================================\n");
fprintf("QUINTIC TIME SCALING\n");
fprintf("========================================\n");

fprintf("s(0):        %.6f\n", sQ(1));
fprintf("s(T):        %.6f\n", sQ(end));

fprintf("sDot(0):     %.6f\n", sDotQ(1));
fprintf("sDot(T):     %.6f\n", sDotQ(end));

fprintf("sDDot(0):    %.6f\n", sDDotQ(1));
fprintf("sDDot(T):    %.6f\n", sDDotQ(end));

fprintf("Peak |sDot|:   %.6f 1/s\n", max(abs(sDotQ)));
fprintf("Peak |sDDot|:  %.6f 1/s^2\n", max(abs(sDDotQ)));
fprintf("Peak |sDDDot|: %.6f 1/s^3\n", max(abs(sDDDotQ)));


assert(all(sQ >= -1e-12 & sQ <= 1 + 1e-12), ...
    "Quintic s leaves the interval [0,1].");

assert(all(diff(sQ) >= -1e-12), ...
    "Quintic profile is not monotonic.");


figure

tiledlayout(4,1)

nexttile
plot(tQ,sQ,'LineWidth',1.5)
grid on
ylabel('s')
title('Quintic Time Scaling')

nexttile
plot(tQ,sDotQ,'LineWidth',1.5)
grid on
ylabel('ds/dt [1/s]')

nexttile
plot(tQ,sDDotQ,'LineWidth',1.5)
grid on
ylabel('d^2s/dt^2 [1/s^2]')

nexttile
plot(tQ,sDDDotQ,'LineWidth',1.5)
grid on
ylabel('d^3s/dt^3 [1/s^3]')
xlabel('Time [s]')


%% ========================================================================
%  3. TRAPEZOIDAL / TRIANGULAR TIME SCALING
% =========================================================================

% Here we choose maximum normalized path velocity and acceleration.
%
% The function calculates the fastest feasible total motion time.

vMax_trap = 0.4;      % [1/s]
aMax_trap = 0.4;      % [1/s^2]
dt_trap   = 0.01;     % [s]


[tT, sT, sDotT, sDDotT, sDDDotT, infoT] = ...
    trapezoidalTimeScaling( ...
        vMax_trap, ...
        aMax_trap, ...
        dt_trap);


fprintf("\n========================================\n");
fprintf("TRAPEZOIDAL TIME SCALING\n");
fprintf("========================================\n");

fprintf("Profile type:      %s\n", infoT.type);
fprintf("Total time:        %.6f s\n", infoT.T);

fprintf("Requested vMax:    %.6f 1/s\n", vMax_trap);
fprintf("Actual peak speed: %.6f 1/s\n", infoT.vPeak);

fprintf("Requested aMax:    %.6f 1/s^2\n", aMax_trap);

fprintf("Acceleration time: %.6f s\n", infoT.tAccel);
fprintf("Cruise time:       %.6f s\n", infoT.tCruise);


assert(all(sT >= -1e-12 & sT <= 1 + 1e-12), ...
    "Trapezoidal s leaves the interval [0,1].");

assert(all(diff(sT) >= -1e-12), ...
    "Trapezoidal profile is not monotonic.");

assert(max(abs(sDotT)) <= vMax_trap + 1e-10, ...
    "Trapezoidal velocity exceeds requested limit.");

assert(max(abs(sDDotT)) <= aMax_trap + 1e-10, ...
    "Trapezoidal acceleration exceeds requested limit.");


figure

tiledlayout(4,1)

nexttile
plot(tT,sT,'LineWidth',1.5)
grid on
ylabel('s')
title("Trapezoidal Time Scaling - " + infoT.type)

nexttile
plot(tT,sDotT,'LineWidth',1.5)
grid on
ylabel('ds/dt [1/s]')

nexttile
plot(tT,sDDotT,'LineWidth',1.5)
grid on
ylabel('d^2s/dt^2 [1/s^2]')

nexttile
plot(tT,sDDDotT,'LineWidth',1.5)
grid on
ylabel('d^3s/dt^3 [1/s^3]')
xlabel('Time [s]')


%% ========================================================================
%  4. S-CURVE TIME SCALING
% =========================================================================

vMax_scurve = 0.4;     % [1/s]
aMax_scurve = 0.4;     % [1/s^2]
jMax_scurve = 1.0;     % [1/s^3]
dt_scurve   = 0.01;    % [s]


[tS, sS, sDotS, sDDotS, sDDDotS, infoS] = ...
    sCurveTimeScaling( ...
        vMax_scurve, ...
        aMax_scurve, ...
        jMax_scurve, ...
        dt_scurve);


fprintf("\n========================================\n");
fprintf("S-CURVE TIME SCALING\n");
fprintf("========================================\n");

fprintf("Total time:          %.6f s\n", infoS.T);

fprintf("Requested vMax:      %.6f 1/s\n", vMax_scurve);
fprintf("Actual peak speed:   %.6f 1/s\n", infoS.vPeak);

fprintf("Requested aMax:      %.6f 1/s^2\n", aMax_scurve);
fprintf("Actual peak accel:   %.6f 1/s^2\n", infoS.aPeak);

fprintf("Requested jMax:      %.6f 1/s^3\n", jMax_scurve);

fprintf("Jerk-ramp time:      %.6f s\n", infoS.tJerk);
fprintf("Constant-accel time: %.6f s\n", ...
    infoS.tConstantAcceleration);

fprintf("Cruise time:         %.6f s\n", ...
    infoS.tConstantVelocity);


assert(all(sS >= -1e-12 & sS <= 1 + 1e-12), ...
    "S-curve s leaves the interval [0,1].");

assert(all(diff(sS) >= -1e-12), ...
    "S-curve profile is not monotonic.");

assert(max(abs(sDotS)) <= vMax_scurve + 1e-10, ...
    "S-curve velocity exceeds requested limit.");

assert(max(abs(sDDotS)) <= aMax_scurve + 1e-10, ...
    "S-curve acceleration exceeds requested limit.");

assert(max(abs(sDDDotS)) <= jMax_scurve + 1e-10, ...
    "S-curve jerk exceeds requested limit.");


figure

tiledlayout(4,1)

nexttile
plot(tS,sS,'LineWidth',1.5)
grid on
ylabel('s')
title('S-Curve Time Scaling')

nexttile
plot(tS,sDotS,'LineWidth',1.5)
grid on
ylabel('ds/dt [1/s]')

nexttile
plot(tS,sDDotS,'LineWidth',1.5)
grid on
ylabel('d^2s/dt^2 [1/s^2]')

nexttile
plot(tS,sDDDotS,'LineWidth',1.5)
grid on
ylabel('d^3s/dt^3 [1/s^3]')
xlabel('Time [s]')


%% ========================================================================
%  5. SUMMARY
% =========================================================================

fprintf("\n========================================\n");
fprintf("TIME-SCALING TEST SUMMARY\n");
fprintf("========================================\n");

fprintf("Cubic T:        %.3f s\n", infoC.T);
fprintf("Quintic T:      %.3f s\n", infoQ.T);
fprintf("Trapezoidal T:  %.3f s\n", infoT.T);
fprintf("S-Curve T:      %.3f s\n", infoS.T);

fprintf("========================================\n");