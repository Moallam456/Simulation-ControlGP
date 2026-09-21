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
results.transformChain = validateTransformChain(robot);
results.massProperties = validateMassProperties(robot);
results.fkVsRigidBodyTree = validateFKAgainstRigidBodyTree(robot,numTests);
results.ik = validateIK(robot,numTests);
results.geometryPropagation = validateGeometryPropagation(robot);

results.success = ...
    results.structure.success && ...
    results.parameters.success && ...
    results.transformChain.success && ...
    results.massProperties.success && ...
    results.fkVsRigidBodyTree.success && ...
    results.ik.success && ...
    results.geometryPropagation.success;

fprintf('\n=====================================\n');
fprintf('OVERALL SUCCESS: %d\n',results.success);
fprintf('=====================================\n');

end

function result = validateStructure(robot)

expectedBodies = robot.structure.dof + 1 + ...
    double(isfield(robot.structure.frames,'baseStructure'));
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

function result = validateTransformChain(robot)

tolerance = 1e-12;
maxFixedTransformError = 0;
maxVisualEndpointError = 0;

T = eye(4);

for i = 1:robot.structure.dof
    T = T * robot.chain.fixedTransforms{i};
    maxFixedTransformError = max(maxFixedTransformError, ...
        norm(T - robot.chain.homeFrames{i+1},'fro'));

    if isfield(robot.chain,'visualSegments') && numel(robot.chain.visualSegments) >= i
        endpointParent = robot.chain.visualSegments{i}(end,:);
        endpointWorld = transformPoint(robot.chain.homeFrames{i},endpointParent);
        frameWorld = robot.chain.homeFrames{i+1}(1:3,4).';
        maxVisualEndpointError = max(maxVisualEndpointError, ...
            norm(endpointWorld - frameWorld));
    end
end

result.maxFixedTransformError = maxFixedTransformError;
result.maxVisualEndpointError = maxVisualEndpointError;
result.success = maxFixedTransformError < tolerance && ...
    maxVisualEndpointError < tolerance;

fprintf('Transform chain consistent: %d\n',result.success);
fprintf('Visual endpoint max error: %.3e m\n',maxVisualEndpointError);

end

function pointWorld = transformPoint(T,pointLocal)

pointHomogeneous = T * [pointLocal(:); 1];
pointWorld = pointHomogeneous(1:3).';

end

function result = validateMassProperties(robot)

bodyCount = numel(robot.model.Bodies);
mass = zeros(bodyCount,1);
comFinite = true(bodyCount,1);
inertiaFinite = true(bodyCount,1);
inertiaNonnegative = true(bodyCount,1);

for i = 1:bodyCount
    body = robot.model.Bodies{i};
    mass(i) = body.Mass;
    comFinite(i) = all(isfinite(body.CenterOfMass));
    inertiaFinite(i) = all(isfinite(body.Inertia));
    inertiaNonnegative(i) = all(body.Inertia(1:3) >= 0);
end

masslessNames = string(robot.model.BodyNames(mass == 0));
result.nonnegativeMass = all(mass >= 0);
result.masslessFrameOnly = isempty(masslessNames) || ...
    (numel(masslessNames) == 1 && ...
    masslessNames == string(robot.structure.frames.flange));
result.finiteCOM = all(comFinite);
result.finiteInertia = all(inertiaFinite);
result.nonnegativeInertiaDiagonal = all(inertiaNonnegative);
result.success = result.nonnegativeMass && result.masslessFrameOnly && ...
    result.finiteCOM && ...
    result.finiteInertia && ...
    result.nonnegativeInertiaDiagonal;

fprintf('Mass properties valid: %d\n',result.success);

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

positionTolerance = 1e-5;
orientationTolerance = 1e-4;
limits = robot.params.joints.positionLimits;
seed = robot.params.joints.homePosition;
prior = rng;
restore = onCleanup(@() rng(prior)); %#ok<NASGU>
rng(1,'twister');
passed = 0;
maxPositionError = 0;
maxOrientationError = 0;
for k = 1:numTests
    target = forward_kinematics(robot,random_configuration(robot));
    solutions = robot.solveIK(target,struct('qSeed',seed));
    if ~solutions.valid || size(solutions.q,2)~=robot.structure.dof
        continue;
    end
    allValid = true;
    distances = vecnorm(solutions.q-seed,2,2);
    allValid = allValid && all(diff(distances)>=-1e-10) && ...
        all(all(solutions.q>=limits(:,1).'-1e-9 & ...
        solutions.q<=limits(:,2).'+1e-9));
    for i = 1:size(solutions.q,1)
        achieved = forward_kinematics(robot,solutions.q(i,:));
        positionError = norm(target(1:3,4)-achieved(1:3,4));
        RError = target(1:3,1:3).'*achieved(1:3,1:3);
        orientationError = acos(max(-1,min(1,(trace(RError)-1)/2)));
        maxPositionError = max(maxPositionError,positionError);
        maxOrientationError = max(maxOrientationError,orientationError);
        allValid = allValid && positionError<=positionTolerance && ...
            orientationError<=orientationTolerance;
        if i>1
            allValid = allValid && ...
                all(vecnorm(solutions.q(1:i-1,:)-solutions.q(i,:),2,2)>=1e-6);
        end
    end
    if allValid, passed = passed+1; end
end
exampleQ = deg2rad([0 20 -30 0 20 0]);
examplePose = forward_kinematics(robot,exampleQ);
example = robot.solveIK(examplePose,struct('qSeed',seed));
result.multipleSolutionCount = size(example.q,1);
unreachable = examplePose;
unreachable(1,4) = unreachable(1,4)+10;
outside = robot.solveIK(unreachable,struct('qSeed',seed));
result.unreachableRejected = ~outside.valid && isempty(outside.q);
halfTurn = examplePose;
halfTurn(1:3,1:3) = examplePose(1:3,1:3)*diag([-1 -1 1]);
halfTurnSolutions = robot.solveIK(halfTurn,struct('qSeed',seed));
result.halfTurnValidated = ~halfTurnSolutions.valid;
if halfTurnSolutions.valid
    result.halfTurnValidated = true;
    for i=1:size(halfTurnSolutions.q,1)
        achieved = forward_kinematics(robot,halfTurnSolutions.q(i,:));
        RError = halfTurn(1:3,1:3).'*achieved(1:3,1:3);
        angleError = acos(max(-1,min(1,(trace(RError)-1)/2)));
        result.halfTurnValidated = result.halfTurnValidated && ...
            norm(halfTurn(1:3,4)-achieved(1:3,4))<=positionTolerance && ...
            angleError<=orientationTolerance;
    end
end
numerical = robot.solveIK(examplePose,struct( ...
    'qSeed',seed,'method',"numerical",'numStarts',2));
result.numericalFallbackValid = numerical.valid && ...
    numerical.info.method=="numerical_multistart";
if result.numericalFallbackValid
    numericalPose = forward_kinematics(robot,numerical.q(1,:));
    result.numericalFallbackValid = ...
        norm(numericalPose(1:3,4)-examplePose(1:3,4))<=positionTolerance;
end

generator = analyticalInverseKinematics(robot.model);
generator.KinematicGroup = struct('BaseName', ...
    char(robot.structure.frames.base),'EndEffectorBodyName', ...
    char(robot.structure.frames.endEffector));
result.wristMetadataCorrect = robot.structure.hasSphericalWrist && ...
    generator.IsValidGroupForIK;

changedParams = robot.params;
changedParams.geometry.l2 = changedParams.geometry.l2+0.05;
changedRobot = loadRobot(robot.id,changedParams);
changedPose = forward_kinematics(changedRobot,exampleQ);
changedSolution = changedRobot.solveIK(changedPose,struct('qSeed',seed));
result.changedGeometryValid = changedSolution.valid;
if result.changedGeometryValid
    changedAchieved = forward_kinematics(changedRobot,changedSolution.q(1,:));
    result.changedGeometryValid = ...
        norm(changedAchieved(1:3,4)-changedPose(1:3,4))<=positionTolerance;
end

result.numTests = numTests;
result.passed = passed;
result.maxPositionError = maxPositionError;
result.maxOrientationError = maxOrientationError;
result.success = passed==numTests && result.multipleSolutionCount>=2 && ...
    result.unreachableRejected && result.halfTurnValidated && ...
    result.numericalFallbackValid && ...
    result.wristMetadataCorrect && ...
    result.changedGeometryValid;
fprintf('IK FK-check passed: %d / %d\n',passed,numTests);
fprintf('IK example solutions: %d; unreachable rejected: %d\n', ...
    result.multipleSolutionCount,result.unreachableRejected);

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
