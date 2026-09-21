function result = test_one_link_dynamics()
% TEST_ONE_LINK_DYNAMICS Hand check: Y-axis pivot, COM along +X, g along -Z.
% r x (m*g) points +Y; holding torque is its opposite: -m*g*r.
mass = 2.0;
radius = 0.4;
gravity = 9.81;
model = rigidBodyTree('DataFormat','row');
model.Gravity = [0 0 -gravity];
body = rigidBody('pendulum');
joint = rigidBodyJoint('pivot','revolute');
joint.JointAxis = [0 1 0];
body.Joint = joint;
body.Mass = mass;
body.CenterOfMass = [radius 0 0];
% I_COM = diag([0.02 0.02 0.02]); shift m*r^2 to Y and Z at body origin.
body.Inertia = [0.02 0.02+mass*radius^2 0.02+mass*radius^2 0 0 0];
addBody(model,body,'base');
expected = -mass*gravity*radius;
actual = gravityTorque(model,0);
result.expected_Nm = expected;
result.actual_Nm = actual;
result.error_Nm = abs(actual-expected);
result.expectedMassMatrix_kg_m2 = 0.02 + mass*radius^2;
result.actualMassMatrix_kg_m2 = massMatrix(model,0);
result.massMatrixError_kg_m2 = abs( ...
    result.actualMassMatrix_kg_m2-result.expectedMassMatrix_kg_m2);
result.accelerationTorque_Nm = ...
    inverseDynamics(model,0,0,1)-gravityTorque(model,0);
result.accelerationTorqueError_Nm = abs( ...
    result.accelerationTorque_Nm-result.expectedMassMatrix_kg_m2);
result.pass = result.error_Nm < 1e-10 && ...
    result.massMatrixError_kg_m2 < 1e-10 && ...
    result.accelerationTorqueError_Nm < 1e-10;
end
