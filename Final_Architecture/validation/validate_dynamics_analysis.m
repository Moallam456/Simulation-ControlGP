function report = validate_dynamics_analysis(robot)
% VALIDATE_DYNAMICS_ANALYSIS Software identities and initial_trial smoke tests.
modelCheck = validateDynamicModel(robot);
if ~modelCheck.pass
    error('validation:InvalidDynamicModel','%s',strjoin(modelCheck.errors,newline));
end
report.model = modelCheck;
originalGravity = robot.model.Gravity;
report.numerical = verifyDynamicsConsistency(robot,struct( ...
    'gravity',[0 0 -9.81],'numSamples',12,'seed',19));
report.oneLink = test_one_link_dynamics();
q0 = robot.params.joints.homePosition;
report.gravity = analyzeGravityLoading(robot,struct( ...
    'gravity',[0 0 -9.81],'q',q0,'numSamples',20,'seed',19));
t = [0; 0.3; 1.1; 2.0];
q = repmat(q0,4,1);
trajectory = struct('time',t,'q',q,'qd',zeros(size(q)),'qdd',zeros(size(q)));
report.staticTrajectory = analyzeTrajectoryDynamics(robot,trajectory, ...
    struct('gravity',[0 0 -9.81]));
expectedStatic = repmat(report.gravity.torque(1,:),4,1);
report.staticTorqueError_Nm = max(abs( ...
    report.staticTrajectory.torque.total(:)-expectedStatic(:)));
report.gravityMoment_Nm = estimateGravityMomentFromCom(robot,q0,[0 0 -9.81]);
report.gravityMomentError_Nm = max(abs(report.gravityMoment_Nm- ...
    report.gravity.torque(1,:)));
report.originalGravity = originalGravity;
report.gravityNotMutated = isequal(robot.model.Gravity,originalGravity);
paramsB = robot.params;
paramsB.geometry.l4 = paramsB.geometry.l4 + 0.03;
robotB = loadRobot(robot.id,paramsB);
qPose = q0;
qPose(2) = deg2rad(40);
tauA = gravityTorqueWithScenario(robot,qPose,[0 0 -9.81]);
tauB = gravityTorqueWithScenario(robotB,qPose,[0 0 -9.81]);
report.geometryTorqueChange_Nm = max(abs(tauB-tauA));
report.geometryPropagates = report.geometryTorqueChange_Nm > 1e-6;
synthetic.time = [0;1;3];
synthetic.state.q = zeros(3,robot.structure.dof);
synthetic.state.qd = zeros(3,robot.structure.dof);
synthetic.state.qdd = zeros(3,robot.structure.dof);
synthetic.torque.total = repmat([0;2;0],1,robot.structure.dof);
synthetic.torque.gravity = zeros(3,robot.structure.dof);
synthetic.torque.inertial = zeros(3,robot.structure.dof);
synthetic.torque.velocity = zeros(3,robot.structure.dof);
synthetic.power.joint = zeros(3,robot.structure.dof);
syntheticSummary = summarizeDynamicsResults(synthetic,robot);
report.nonuniformRmsError_Nm = max(abs( ...
    syntheticSummary.RMSTorqueOverTrajectory_Nm-sqrt(2)));
paramsOverride = robot.params;
paramsOverride.links(3).centerOfMass = [0 0 -0.175];
paramsOverride.links(3).inertia = diag([0.05 0.05 0.01]);
paramsOverride.links(3).dynamics.status = "CAD";
paramsOverride.tool.inertia = diag([0.01 0.02 0.03]);
paramsOverride.tool.dynamics.status = "CAD";
robotOverride = loadRobot(robot.id,paramsOverride);
overrideCheck = validateDynamicModel(robotOverride);
report.explicitInertiaMapping = overrideCheck.pass && ...
    abs(robotOverride.model.Bodies{3}.Inertia(1)-0.05) < 1e-12 && ...
    ~isequal(robotOverride.model.Bodies{end}.Inertia,robot.model.Bodies{end}.Inertia);
report.pass = report.numerical.pass && report.oneLink.pass && ...
    report.staticTorqueError_Nm < 1e-9 && ...
    report.gravityMomentError_Nm < 1e-9 && ...
    report.gravityNotMutated && report.geometryPropagates && ...
    report.nonuniformRmsError_Nm < 1e-12 && ...
    report.explicitInertiaMapping;
fprintf('Dynamics model: %s\n',report.staticTrajectory.meta.label);
fprintf('Max decomposition residual: %.3e N*m\n',report.numerical.maxDecompositionResidual_Nm);
fprintf('Max forward/inverse acceleration error: %.3e rad/s^2\n',report.numerical.maxForwardInverseError_rad_s2);
fprintf('Max mass-matrix asymmetry: %.3e kg*m^2\n',report.numerical.maxMassMatrixAsymmetry_kg_m2);
fprintf('Min mass-matrix eigenvalue: %.3e kg*m^2\n',report.numerical.minMassMatrixEigenvalue_kg_m2);
fprintf('One-link gravity expected/actual: %.6f / %.6f N*m\n', ...
    report.oneLink.expected_Nm,report.oneLink.actual_Nm);
fprintf('COM lever-arm gravity check error: %.3e N*m\n',report.gravityMomentError_Nm);
fprintf('Geometry +0.03 m torque change: %.3e N*m\n',report.geometryTorqueChange_Nm);
fprintf('Nonuniform-time RMS check error: %.3e N*m\n',report.nonuniformRmsError_Nm);
fprintf('Explicit COM/inertia mapping valid: %d\n',report.explicitInertiaMapping);
end

function tau = gravityTorqueWithScenario(robot,q,gravity)
model = copy(robot.model);
model.Gravity = gravity;
tau = gravityTorque(model,q);
end
