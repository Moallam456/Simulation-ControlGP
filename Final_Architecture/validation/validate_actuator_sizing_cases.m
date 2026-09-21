function results = validate_actuator_sizing_cases(robot)
% VALIDATE_ACTUATOR_SIZING_CASES Geometry, timing, IK, FK, and case integration.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(root,fullfile(root,'kinematics'),fullfile(root,'trajectory'), ...
    fullfile(root,'actuator_sizing_cases'), ...
    fullfile(root,'dynamics_analysis'));
if nargin<1, robot=loadRobot(); end
cases=createActuatorSizingCases(robot);
line=cases(2);
T=forward_kinematics(robot,line.startConfiguration);
line.path.startPosition=T(1:3,4);
line.path.startOrientation=T(1:3,1:3);
g=generatePathGeometry(line.path);
[~,p]=evaluatePathGeometry(g,linspace(0,1,11));
assert(norm(p(1,:).'-line.path.startPosition)<1e-12);
assert(norm(p(end,:).'-line.path.endPosition)<1e-12);
assert(max(vecnorm(cross(p-p(1,:),repmat(p(end,:)-p(1,:),11,1),2),2,2))<1e-12);
rotated=line.path;
rotated.orientationMode='slerp';
rotated.endOrientation=axang2rotm([0 0 1 pi/3])*T(1:3,1:3);
g=generatePathGeometry(rotated);
[poses,~]=evaluatePathGeometry(g,[0 .5 1]);
assert(norm(poses(1:3,1:3,1)-T(1:3,1:3),'fro')<1e-10);
assert(norm(poses(1:3,1:3,end)-rotated.endOrientation,'fro')<1e-10);
assert(norm(poses(1:3,1:3,2)'*poses(1:3,1:3,2)-eye(3),'fro')<1e-10);
arc=cases(3).path;
arc.startPosition=T(1:3,4); arc.startOrientation=T(1:3,1:3);
g=generatePathGeometry(arc);
[~,p]=evaluatePathGeometry(g,linspace(0,1,101));
assert(max(abs(vecnorm(p-g.center.',2,2)-g.radius))<1e-10);
assert(norm(p(1,:).'-arc.startPosition)<1e-12);
assert(norm(p(end,:).'-arc.endPosition)<1e-12);
assert(min(vecnorm(p-arc.midPosition(:).',2,2)) < g.length/100);
circle=cases(4).path;
circle.startPosition=T(1:3,4); circle.startOrientation=T(1:3,1:3);
g=generatePathGeometry(circle);
[poses,p]=evaluatePathGeometry(g,linspace(0,1,101));
assert(abs(abs(g.sweep)-2*pi)<1e-12);
assert(norm(p(1,:)-p(end,:))<1e-10);
assert(norm(poses(1:3,1:3,1)-poses(1:3,1:3,end),'fro')<1e-10);
assert(max(abs(vecnorm(p-g.center.',2,2)-g.radius))<1e-10);
profile=timeScalingProfile(g.length,cases(4).motion);
assert(abs(profile.s(1))<1e-12 && abs(profile.s(end)-1)<1e-10);
assert(all(diff(profile.s)>=-1e-10));
assert(max(profile.tcpSpeed)<=cases(4).motion.tcpSpeed+1e-9);
assert(max(abs(profile.tcpAcceleration))<=cases(4).motion.tcpAcceleration+1e-9);
assert(max(abs(profile.tcpJerk))<=cases(4).motion.tcpJerk+1e-9);
scenario.gravity=[0 0 -9.81];
options=struct('checkConvergence',true,'gravitySamples',25, ...
    'gravityStarts',1,'gravityMaxEvaluations',100);
suite=runActuatorSizingCases(robot,cases,scenario,options);
assert(suite.cases(1).status=="PASS" && ~isempty(suite.gravity));
required=["LINE_WELD" "ARC_WELD" "FULL_CIRCLE_WELD" "MULTI_SEGMENT"];
for id=required
    i=find(string({suite.cases.id})==id,1);
    assert(suite.cases(i).status=="PASS", ...
        '%s failed: %s',id,suite.cases(i).reason);
    tr=suite.cases(i).trajectory;
    assert(size(tr.q,1)==numel(tr.time));
    assert(size(tr.q,2)==robot.structure.dof);
    assert(all(isfinite([tr.q(:);tr.qd(:);tr.qdd(:)])));
    assert(suite.cases(i).convergence.pass);
    ik=suite.cases(i).trajectoryReport.ikReports{1};
    assert(ik.pass && ik.maxJointStep_rad<=cases(i).validation.maxSpatialJointStepRad);
    if id=="LINE_WELD"
        assert(any(ik.candidateCounts(2:end)>1));
    end
    if id=="MULTI_SEGMENT"
        waypoints=cases(i).path.waypoints;
        junctions=cumsum(vecnorm(diff(waypoints,1,1),2,2));
        for distance=junctions(1:end-1).'
            index=find(abs(tr.cartesian.arcLength-distance)<1e-9,1);
            assert(~isempty(index));
            assert(max(abs(tr.qd(index,:)))<1e-8);
            assert(max(abs(tr.qdd(index,:)))<1e-6);
        end
    end
end
assert(numel(suite.aggregate.validCaseIDs)==numel(required));
for j=1:robot.structure.dof
    points=suite.aggregate.torqueSpeed(j);
    assert(numel(points.speed_rad_s)==numel(points.torque_Nm));
    assert(numel(points.time_s)==numel(points.caseID));
end
results.pass=true;
results.suite=suite;
fprintf('Actuator sizing case validation: PASS\n');
end
