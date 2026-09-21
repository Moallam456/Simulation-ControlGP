clear;
clc;

%% ========================================================================
% MATLAB <-> C SINGLE-SEGMENT LINE PARITY TEST
% =========================================================================

scriptFolder = fileparts(mfilename('fullpath'));

matlabFile = fullfile( ...
    scriptFolder, ...
    'single_segment_line_matlab.csv');

cFile = fullfile( ...
    scriptFolder, ...
    'single_segment_line_c.csv');


%% ========================================================================
% LOAD DATA
% =========================================================================

M = readtable(matlabFile);
C = readtable(cFile);

fprintf('\n');
fprintf('============================================================\n');
fprintf(' MATLAB <-> C NUMERICAL PARITY TEST\n');
fprintf('============================================================\n');

fprintf('MATLAB samples: %d\n', height(M));
fprintf('C samples:      %d\n', height(C));

if height(M) ~= height(C)
    error('Sample counts do not match.');
end

fprintf('Sample count:   PASS\n');


%% ========================================================================
% TIME SCALING
% =========================================================================

timeError   = max(abs(M.time   - C.time));
sError      = max(abs(M.s      - C.s));
sDotError   = max(abs(M.sDot   - C.sDot));
sDDotError  = max(abs(M.sDDot  - C.sDDot));
sDDDotError = max(abs(M.sDDDot - C.sDDDot));

fprintf('\nTIME SCALING\n');
fprintf('------------------------------------------------------------\n');
fprintf('Max time difference:       %.12e s\n', timeError);
fprintf('Max s difference:          %.12e\n', sError);
fprintf('Max sDot difference:       %.12e\n', sDotError);
fprintf('Max sDDot difference:      %.12e\n', sDDotError);
fprintf('Max sDDDot difference:     %.12e\n', sDDDotError);


%% ========================================================================
% ARC LENGTH + TCP SPEED
% =========================================================================

arcError = max(abs( ...
    M.arcLength - C.arcLength));

tcpSpeedError = max(abs( ...
    M.tcpSpeed - C.tcpSpeed));

fprintf('\nARC LENGTH / TCP SPEED\n');
fprintf('------------------------------------------------------------\n');
fprintf('Max arc-length difference: %.12e m\n', arcError);
fprintf('Max TCP speed difference:  %.12e m/s\n', tcpSpeedError);


%% ========================================================================
% CARTESIAN POSITION
% =========================================================================

pM = [M.px M.py M.pz];
pC = [C.px C.py C.pz];

pDifference = pM - pC;

cartesianDistanceDifference = ...
    sqrt(sum(pDifference.^2, 2));

maxCartesianDifference = ...
    max(cartesianDistanceDifference);

maxXYZComponentDifference = ...
    max(abs(pDifference), [], 'all');

fprintf('\nCARTESIAN PATH\n');
fprintf('------------------------------------------------------------\n');
fprintf('Max XYZ component diff:    %.12e m\n', ...
    maxXYZComponentDifference);

fprintf('Max Cartesian distance:    %.12e m\n', ...
    maxCartesianDifference);


%% ========================================================================
% ORIENTATION
% =========================================================================
%
% Quaternion q and -q represent the same orientation.
% Therefore compare physical angular difference instead of raw components.
% =========================================================================

quatM = [M.qw M.qx M.qy M.qz];
quatC = [C.qw C.qx C.qy C.qz];

quatM = quatM ./ vecnorm(quatM, 2, 2);
quatC = quatC ./ vecnorm(quatC, 2, 2);

quatDot = sum(quatM .* quatC, 2);

quatDot = abs(quatDot);
quatDot = min(1, max(-1, quatDot));

orientationDifference = ...
    2 * acos(quatDot);

maxOrientationDifference = ...
    max(orientationDifference);

fprintf('\nDESIRED ORIENTATION\n');
fprintf('------------------------------------------------------------\n');
fprintf('Max orientation difference: %.12e rad\n', ...
    maxOrientationDifference);

fprintf('Max orientation difference: %.12e deg\n', ...
    rad2deg(maxOrientationDifference));


%% ========================================================================
% JOINT TRAJECTORY
% =========================================================================

qM = [ ...
    M.q1 M.q2 M.q3 ...
    M.q4 M.q5 M.q6];

qC = [ ...
    C.q1 C.q2 C.q3 ...
    C.q4 C.q5 C.q6];

qDifference = abs(qM - qC);

maxJointDifference = ...
    max(qDifference, [], 'all');

maxJointDifferenceByJoint = ...
    max(qDifference, [], 1);

fprintf('\nJOINT TRAJECTORY q(t)\n');
fprintf('------------------------------------------------------------\n');

fprintf('Maximum joint difference:  %.12e rad\n', ...
    maxJointDifference);

fprintf('Maximum joint difference:  %.12e deg\n', ...
    rad2deg(maxJointDifference));

for joint = 1:6

    fprintf( ...
        'J%d max difference:       %.12e rad  (%.12e deg)\n', ...
        joint, ...
        maxJointDifferenceByJoint(joint), ...
        rad2deg(maxJointDifferenceByJoint(joint)));

end


%% ========================================================================
% JOINT VELOCITY
% =========================================================================

qdM = [ ...
    M.qd1 M.qd2 M.qd3 ...
    M.qd4 M.qd5 M.qd6];

qdC = [ ...
    C.qd1 C.qd2 C.qd3 ...
    C.qd4 C.qd5 C.qd6];

qdDifference = abs(qdM - qdC);

maxVelocityDifference = ...
    max(qdDifference, [], 'all');

fprintf('\nJOINT VELOCITY qDot(t)\n');
fprintf('------------------------------------------------------------\n');

fprintf('Maximum qDot difference:   %.12e rad/s\n', ...
    maxVelocityDifference);


%% ========================================================================
% FK VALIDATION
% =========================================================================

positionErrorDifference = ...
    max(abs( ...
        M.positionError - ...
        C.positionError));

orientationErrorDifference = ...
    max(abs( ...
        M.orientationError - ...
        C.orientationError));

fprintf('\nFK VALIDATION\n');
fprintf('------------------------------------------------------------\n');

fprintf('MATLAB max position error: %.12e m\n', ...
    max(M.positionError));

fprintf('C max position error:      %.12e m\n', ...
    max(C.positionError));

fprintf('Max position-error diff:   %.12e m\n', ...
    positionErrorDifference);

fprintf('\n');

fprintf('MATLAB max orientation err: %.12e rad\n', ...
    max(M.orientationError));

fprintf('C max orientation error:    %.12e rad\n', ...
    max(C.orientationError));

fprintf('Max orientation-error diff: %.12e rad\n', ...
    orientationErrorDifference);


%% ========================================================================
% IK ITERATIONS
% =========================================================================

iterationDifference = ...
    abs(M.ikIterations - C.ikIterations);

fprintf('\nIK ITERATIONS\n');
fprintf('------------------------------------------------------------\n');

fprintf('MATLAB maximum iterations: %d\n', ...
    max(M.ikIterations));

fprintf('C maximum iterations:      %d\n', ...
    max(C.ikIterations));

fprintf('Largest iteration diff:    %d\n', ...
    max(iterationDifference));


%% ========================================================================
% AUTOMATIC PARITY SUMMARY
% =========================================================================

frontEndTolerance = 1e-9;

frontEndPass = ...
    timeError                  < frontEndTolerance && ...
    sError                     < frontEndTolerance && ...
    sDotError                  < frontEndTolerance && ...
    sDDotError                 < frontEndTolerance && ...
    sDDDotError                < frontEndTolerance && ...
    arcError                   < frontEndTolerance && ...
    tcpSpeedError              < frontEndTolerance && ...
    maxCartesianDifference     < frontEndTolerance && ...
    maxOrientationDifference   < 1e-7;

fprintf('\n');
fprintf('============================================================\n');

if frontEndPass
    fprintf(' FRONT-END TRAJECTORY PARITY: PASS\n');
else
    fprintf(' FRONT-END TRAJECTORY PARITY: CHECK REQUIRED\n');
end

fprintf('------------------------------------------------------------\n');

fprintf('Max joint trajectory difference: %.12e rad\n', ...
    maxJointDifference);

fprintf('Max joint trajectory difference: %.12e deg\n', ...
    rad2deg(maxJointDifference));

fprintf('============================================================\n');