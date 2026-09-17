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
body.Inertia = [0.02 0.02 0.02 0 0 0];
addBody(model,body,'base');
expected = -mass*gravity*radius;
actual = gravityTorque(model,0);
result.expected_Nm = expected;
result.actual_Nm = actual;
result.error_Nm = abs(actual-expected);
result.pass = result.error_Nm < 1e-10;
end
