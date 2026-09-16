function results = validate_kinematics(robot,numTests)

% VALIDATE_KINEMATICS Validate the loaded robot interface and kinematics.
%
% Preferred:
%   robot = loadRobot();
%   results = validate_kinematics(robot);

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(rootDir,fullfile(rootDir,'kinematics'),fullfile(rootDir,'validation'));

if nargin < 1 || isempty(robot)
    robot = loadRobot();
end

if nargin < 2 || isempty(numTests)
    numTests = 100;
end

fprintf('\n=====================================\n');
fprintf('KINEMATICS VALIDATION: %s\n',robot.id);
fprintf('=====================================\n\n');

results.structure = validateStructure(robot);
results.parameters = validateParameters(robot);
results.fkVsRigidBodyTree = validateFKAgainstRigidBodyTree(robot,numTests);
results.ik = validateIK(robot,numTests);
results.geometryPropagation = validateGeometryPropagation(robot);

results.success = ...
    results.structure.success && ...
    results.parameters.success && ...
    results.fkVsRigidBodyTree.success && ...
    results.ik.success && ...
    results.geometryPropagation.success;

fprintf('\n=====================================\n');
fprintf('OVERALL SUCCESS: %d\n',results.success);
fprintf('=====================================\n');

end

function result = validateStructure(robot)

expectedBodies = robot.structure.dof + 1;
bodyCount = numel(robot.model.Bodies);

result.bodyCount = bodyCount;
result.expectedBodies = expectedBodies;
result.jointAxesFinite = all(isfinite(robot.structure.jointAxes(:)));
result.jointAxesUnit = all(abs(vecnorm(robot.structure.jointAxes,2,2) - 1) < 1e-12);
result.success = bodyCount == expectedBodies && ...
    result.jointAxesFinite && result.jointAxesUnit;

fprintf('Structure bodies: %d / %d\n',bodyCount,expectedBodies);

end

function result = validateParameters(robot)

limits = robot.params.joints.positionLimits;
home = robot.params.joints.homePosition(:);

result.limitsOrdered = all(limits(:,1) < limits(:,2));
result.homeInsideLimits = all(home >= limits(:,1) & home <= limits(:,2));

geometryValues = struct2array(robot.params.geometry);
result.geometryFinite = all(isfinite(geometryValues));

result.success = result.limitsOrdered && ...
    result.homeInsideLimits && result.geometryFinite;

fprintf('Parameters valid: %d\n',result.success);

end

function result = validateFKAgainstRigidBodyTree(robot,numTests)

positionTolerance = 1e-10;
transformTolerance = 1e-10;

maxPositionError = 0;
maxTransformError = 0;

for k = 1:numTests
    q = random_configuration(robot);

    TCustom = forward_kinematics(robot,q);
    TRBT = getTransform( ...
        robot.model, ...
        q, ...
        char(robot.structure.frames.endEffector));

    maxPositionError = max(maxPositionError, ...
        norm(TCustom(1:3,4) - TRBT(1:3,4)));
    maxTransformError = max(maxTransformError,norm(TCustom - TRBT,'fro'));
end

result.numTests = numTests;
result.maxPositionError = maxPositionError;
result.maxTransformError = maxTransformError;
result.success = maxPositionError < positionTolerance && ...
    maxTransformError < transformTolerance;

fprintf('FK vs RBT max position error: %.3e m\n',maxPositionError);
fprintf('FK vs RBT max transform error: %.3e\n',maxTransformError);

end

function result = validateIK(robot,numTests)

positionTolerance = 1e-4;
orientationTolerance = 1e-4;

passed = 0;
maxPositionError = 0;
maxOrientationError = 0;

for k = 1:numTests
    q = random_configuration(robot);
    TTarget = forward_kinematics(robot,q);

    options.qSeed = q;
    solutions = robot.solveIK(TTarget,options);

    if ~solutions.info.success
        continue;
    end

    TCheck = forward_kinematics(robot,solutions.q(1,:));

    positionError = norm(TTarget(1:3,4) - TCheck(1:3,4));
    RError = TTarget(1:3,1:3)' * TCheck(1:3,1:3);
    orientationError = acos(max(min((trace(RError)-1)/2,1),-1));

    maxPositionError = max(maxPositionError,positionError);
    maxOrientationError = max(maxOrientationError,orientationError);

    if positionError < positionTolerance && orientationError < orientationTolerance
        passed = passed + 1;
    end
end

result.numTests = numTests;
result.passed = passed;
result.maxPositionError = maxPositionError;
result.maxOrientationError = maxOrientationError;
result.success = passed == numTests;

fprintf('IK FK-check passed: %d / %d\n',passed,numTests);

end

function result = validateGeometryPropagation(robot)

paramsA = robot.params;
paramsB = robot.params;
paramsB.geometry.l2 = paramsB.geometry.l2 + 0.05;

robotB = loadRobot(robot.id,paramsB);

q = robot.params.joints.homePosition;
TA = forward_kinematics(robot,q);
TB = forward_kinematics(robotB,q);

deltaZ = TB(3,4) - TA(3,4);

result.changedParameter = "params.geometry.l2";
result.expectedDeltaZ = 0.05;
result.actualDeltaZ = deltaZ;
result.success = abs(deltaZ - 0.05) < 1e-12;

fprintf('Geometry propagation l2 -> TCP Z delta: %.3f m\n',deltaZ);

end
