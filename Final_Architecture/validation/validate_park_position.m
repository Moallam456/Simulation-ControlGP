function report=validate_park_position()
% VALIDATE_PARK_POSITION Objective, analytic physics, geometry and route tests.
root=fileparts(fileparts(mfilename('fullpath'))); addpath(genpath(root));
settings=struct('numSamples',32,'numStarts',2,'seed',31, ...
    'maxFunctionEvaluations',180,'maxIterations',35,'maxSeedCollisionChecks',16, ...
    'checkPaths',true,'useRRT',false,'pathStep_deg',5,'numCandidates',3);
r=fixture(false); before=rng;
p=optimizeParkConfiguration(r,settings);
assert(isequal(before,rng),'Search changed caller RNG.');
assert(p.found && abs(p.reference.torque_Nm+7.848)<1e-10);
assert(abs(p.best.torque_Nm)<1e-3,'Analytic pendulum minimum not found.');
assert(~p.meta.motorRatingsAvailable && all(isnan(p.best.utilization)));
assert(p.best.objective==max(abs(p.best.torque_Nm)));
for c=p.candidates
    q=c.pose.q; exactMinimum=1-.4*sin(q)-.4*abs(sin(q))-.03*abs(cos(q));
    assert(abs(c.pose.plane.minimumZ_m-exactMinimum)<1e-8);
    assert(abs(c.pose.energy_J-2*9.81*(1-.4*sin(q)))<1e-8);
    assert(abs(c.pose.stability.potentialHessian-7.848*sin(q))<1e-6);
    assert(c.pose.collision.hasRequiredClearance && c.pose.plane.abovePlane);
    assert(all([c.paths.passed]));
    for path=c.paths
        assert(max(abs(diff(path.q)),[],'all')<=deg2rad(settings.pathStep_deg)+1e-10);
        assert(norm(path.q(1,:)-p.workingStarts(path.startIndex).q)<1e-10);
        assert(norm(path.q(end,:)-q)<1e-10);
    end
end
repeat=optimizeParkConfiguration(r,settings);
assert(norm(repeat.best.q-p.best.q)<1e-9,'Search is not deterministic.');
settings.allowableHoldingTorque_Nm=2;
normalized=optimizeParkConfiguration(r,settings);
assert(normalized.meta.motorRatingsAvailable);
for c=normalized.candidates
    assert(norm(c.pose.utilization-abs(c.pose.torque_Nm)/2)<1e-12);
    assert(abs(c.pose.objective-max(c.pose.utilization))<1e-12);
end
settings.allowableHoldingTorque_Nm=[];
blocked=optimizeParkConfiguration(fixture(true),settings);
assert(~blocked.found && isempty(blocked.best) && ~blocked.optimizationPerformed);
assert(blocked.status=="NO_FEASIBLE_SAMPLED_POSE");
% Joint origin is above the floor, but the long link at +90 degrees is not.
low=fixture(false);
setFixedTransform(low.model.Bodies{1}.Joint,trvec2tform([0 0 .25]));
low.params.joints.homePosition=pi/2;
floorResult=optimizeParkConfiguration(low,settings);
assert(~floorResult.reference.plane.abovePlane && floorResult.found);
assert(~floorResult.workingStarts(1).valid);
for c=floorResult.candidates
    q=c.pose.q; exactMinimum=.25-.4*sin(q)-.4*abs(sin(q))-.03*abs(cos(q));
    assert(exactMinimum>=-1e-6-1e-8);
end
report.analyticTorque_Nm=p.best.torque_Nm;
report.normalizedObjective=true; report.noInventedRatings=true;
report.wholeMeshPlaneCheck=true; report.rejectedCollisions=true;
report.sampledRoutes=true; report.reproducible=true;
settings.checkPaths=false; settings.numSamples=24; settings.numStarts=1;
settings.maxFunctionEvaluations=100; settings.maxIterations=20;
report.models=struct('id',{},'found',{},'seconds',{});
for id=["initial_trial" "robot_ver1" "robot_ver2"]
    robot=loadRobot(id); home=robot.params.joints.homePosition; gravity=robot.model.Gravity;
    result=optimizeParkConfiguration(robot,settings);
    for entry=result.bank
        if startsWith(entry.source,["compact " "j2 " "j3 "])
            assert(entry.objective<=result.meta.alternativePeakLoadCap+1e-6);
        end
    end
    assert(isequal(home,robot.params.joints.homePosition) && isequal(gravity,robot.model.Gravity));
    for c=result.candidates
        model=copy(robot.model); model.Gravity=settingsGravity(result);
        tau=inverseDynamics(model,c.pose.q,zeros(size(home)),zeros(size(home)));
        assert(norm(tau-c.pose.torque_Nm)<1e-9);
        assert(c.pose.plane.abovePlane && c.pose.collision.hasRequiredClearance);
        assert(all(c.pose.q>=robot.params.joints.positionLimits(:,1).') && ...
            all(c.pose.q<=robot.params.joints.positionLimits(:,2).'));
        assert(all(isnan(c.pose.utilization)));
        assert(abs(c.pose.objective-max(abs(tau)))<1e-10);
    end
    report.models(end+1)=struct('id',id,'found',result.found, ...
        'seconds',result.statistics.elapsedSeconds);
    if id=="robot_ver2"
        assert(result.found && result.best.peakAbsTorque_Nm<20);
        assert(result.best.plane.maximumZ_m<=1.35+1e-6);
        assert(result.best.plane.radius_m<=.65+1e-6);
    end
end
report.pass=true; disp(report);
end

function g=settingsGravity(result)
g=result.settings.gravity;
end

function robot=fixture(blocked)
model=rigidBodyTree('DataFormat','row');
body=rigidBody('arm'); body.Joint=rigidBodyJoint('joint_1','revolute');
body.Joint.JointAxis=[0 1 0]; body.Joint.PositionLimits=[-pi pi];
body.Mass=2; body.CenterOfMass=[.4 0 0]; body.Inertia=[.02 .35 .36 0 0 0];
setFixedTransform(body.Joint,trvec2tform([0 0 1]));
addCollision(body,collisionSphere(.05));
addVisual(body,'Box',[.8 .06 .06],trvec2tform([.4 0 0]));
addBody(model,body,model.BaseName);
if blocked
    obstacle=rigidBody('obstacle'); obstacle.Mass=1; obstacle.Inertia=[.01 .01 .01 0 0 0];
    setFixedTransform(obstacle.Joint,trvec2tform([0 0 1]));
    addCollision(obstacle,collisionSphere(.1)); addVisual(obstacle,'Sphere',.1);
    addBody(model,obstacle,model.BaseName);
end
robot.id="analytic_park_test"; robot.model=model;
robot.structure=struct('dof',1,'jointNames',"joint_1",'bodyNames',"arm", ...
    'frames',struct('endEffector',"arm",'flange',"arm"));
robot.params.joints=struct('homePosition',0,'positionLimits',[-pi pi]);
robot.params.links=struct('mass',2,'centerOfMass',[.4 0 0],'inertia',diag([.02 .03 .04]));
end
