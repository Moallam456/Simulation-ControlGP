function report = validate_robot_visuals(robot)
% VALIDATE_ROBOT_VISUALS Check segment ownership at non-home configurations.
if nargin < 1, robot = loadRobot(); end
if ~isfield(robot.chain,'bodySegments') || ...
        ~isfield(robot.structure.frames,'baseStructure')
    error('validation:MissingBodySegments','Robot lacks body-owned geometry.');
end
q0 = robot.params.joints.homePosition;
q3 = q0;
q3(3) = q3(3) + deg2rad(45);
q2 = q0;
q2(2) = q2(2) + deg2rad(45);
poses = [q0;q2;q3;deg2rad([15 30 -25 20 -15 10])];
limits = robot.params.joints.positionLimits;
poses = poses(all(poses >= limits(:,1).' & poses <= limits(:,2).',2),:);
report.maxEndpointError_m = 0;
report.maxFKError_m = 0;
for k = 1:size(poses,1)
    q = poses(k,:);
    [~,frames] = forward_kinematics(robot,q);
    for i = 1:robot.structure.dof-1
        T = getTransform(robot.model,q,char(robot.structure.bodyNames(i)));
        points = robot.chain.bodySegments{i};
        startPoint = T*[points(1,:) 1].';
        endPoint = T*[points(end,:) 1].';
        report.maxEndpointError_m = max(report.maxEndpointError_m, ...
            norm(startPoint(1:3)-frames{i+1}(1:3,4)));
        report.maxEndpointError_m = max(report.maxEndpointError_m, ...
            norm(endPoint(1:3)-frames{i+2}(1:3,4)));
    end
    tcp = getTransform(robot.model,q,char(robot.structure.frames.endEffector));
    report.maxFKError_m = max(report.maxFKError_m, ...
        norm(tcp(1:3,4)-frames{end}(1:3,4)));
end
baseName = char(robot.structure.frames.baseStructure);
baseHome = getTransform(robot.model,q0,baseName);
baseMoved = getTransform(robot.model,q2,baseName);
report.fixedBaseError = norm(baseHome-baseMoved,'fro');
upstreamName = char(robot.structure.bodyNames(2));
downstreamName = char(robot.structure.bodyNames(3));
upstream = getBody(robot.model,upstreamName);
downstream = getBody(robot.model,downstreamName);
report.upstreamCOMMovement_m = norm( ...
    worldCOM(robot.model,q0,upstream) - worldCOM(robot.model,q3,upstream));
report.downstreamCOMMovement_m = norm( ...
    worldCOM(robot.model,q0,downstream) - worldCOM(robot.model,q3,downstream));
report.totalMass_kg = sum(cellfun(@(body) body.Mass,robot.model.Bodies));
expectedMass = robot.params.base.mass + sum([robot.params.links.mass]) + ...
    robot.params.tool.mass + robot.params.tool.segmentMass;
report.massError_kg = abs(report.totalMass_kg-expectedMass);
report.pass = report.maxEndpointError_m < 1e-10 && ...
    report.maxFKError_m < 1e-10 && report.fixedBaseError < 1e-12 && ...
    report.upstreamCOMMovement_m < 1e-12 && ...
    report.downstreamCOMMovement_m > 1e-3 && ...
    report.massError_kg < 1e-10;
fprintf('Body-owned visual geometry: %s (endpoint error %.3e m)\n', ...
    string(report.pass),report.maxEndpointError_m);
end

function p = worldCOM(model,q,body)
T = getTransform(model,q,body.Name);
point = T*[body.CenterOfMass(:);1];
p = point(1:3);
end
