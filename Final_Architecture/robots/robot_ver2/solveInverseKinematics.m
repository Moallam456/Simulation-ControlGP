function solutions = solveInverseKinematics(targetPose,robot,options)
% SOLVEINVERSEKINEMATICS Robot-local control-team closed form, checked by URDF FK.
% Geometry and equations are local helpers; no external source folder is used.

if nargin < 3 || isempty(options)
    options = struct();
end
validateTarget(targetPose);

dof = robot.structure.dof;
limits = robot.params.joints.positionLimits;
qSeed = reshape(option(options,'qSeed',robot.params.joints.homePosition),1,[]);
if numel(qSeed) ~= dof || any(~isfinite(qSeed))
    error('robot_ver2:InvalidIKSeed','qSeed must contain %d finite angles.',dof);
end
qSeed = min(max(qSeed,limits(:,1).'),limits(:,2).');
positionTolerance = option(options,'positionTolerance',1e-5);
orientationTolerance = option(options,'orientationTolerance',1e-4);
if ~isscalar(positionTolerance) || ~isfinite(positionTolerance) || positionTolerance <= 0 || ...
        ~isscalar(orientationTolerance) || ~isfinite(orientationTolerance) || orientationTolerance <= 0
    error('robot_ver2:InvalidIKTolerance','IK tolerances must be positive finite scalars.');
end

P = analyticalGeometry();
P.qmin = limits(:,1).';
P.qmax = limits(:,2).';
Ttool = robot.params.tool.TFlangeTCP;
targetEE = targetPose / Ttool;
[candidates,diagnostic] = analyticalCandidates(targetEE,P);

[q,pError,rError] = validatedCandidates(candidates,robot,targetPose, ...
    limits,positionTolerance,orientationTolerance);
if ~isempty(q)
    delta = atan2(sin(q-qSeed),cos(q-qSeed));
    [~,order] = sort(sum(delta.^2,2));
    q = q(order,:);
    pError = pError(order);
    rError = rError(order);
end

solutions.q = q;
solutions.valid = ~isempty(q);
solutions.branch = repmat("analytical",size(q,1),1);
solutions.info.success = solutions.valid;
solutions.info.numSolutions = size(q,1);
solutions.info.iterations = 0;
solutions.info.method = "analytical_closed_form";
solutions.info.positionErrors = pError;
solutions.info.orientationErrors = rError;
solutions.info.positionError = Inf;
solutions.info.orientationError = Inf;
solutions.info.collisionStatus = ...
    "NOT_FILTERED: mesh collisions require a separate collision check.";
if solutions.valid
    solutions.info.analyticalStatus = "URDF_FK_VALIDATED";
    solutions.info.positionError = pError(1);
    solutions.info.orientationError = rError(1);
    solutions.info.message = "Closed-form IK branches validated against URDF forward kinematics.";
else
    solutions.info.analyticalStatus = "NO_VALIDATED_BRANCH";
    if diagnostic.reason == "ok"
        solutions.info.message = ...
            "Closed-form candidates failed URDF FK, tolerance, or joint-limit checks.";
    else
        solutions.info.message = "No FK-validated, limit-valid closed-form branch: " + diagnostic.reason;
    end
end
end

function P = analyticalGeometry()
% Nominal robot_ver2 geometry, in metres/radians. The URDF is the FK authority.
% Keep these equations and constants together when revising the mechanism.
P.d1 = 0.2000;
P.yP = -0.2425;
P.zA = 0.1320;
P.a2 = 0.589954679438418;
P.a3x = 0.404499960921119;
P.a3y = -4.53236012523117e-5;
P.a3 = hypot(P.a3x,P.a3y);
P.q3Offset = atan2(-P.a3y,P.a3x);
P.d5 = 0.0555;
P.yW = -0.0554999969603261 - 0.1195;
end

function [sols,info] = analyticalCandidates(T,P)
% Control-team RecentIK equations: two base, wrist, and elbow branches.
% The URDF rounds Joint_6's right angle to 1.5708; every branch is FK-checked.
p = T(1:3,4);
R = T(1:3,1:3);
r13 = R(1,3); r23 = R(2,3); r33 = R(3,3);
sols = zeros(0,6);
info.reason = "no closed-form branch found in joint limits";

A = p(1)-P.d5*r13;
B = p(2)-P.d5*r23;
Rn = hypot(A,B);
if Rn < abs(P.yP)
    info.reason = "unreachable: R_xy < |yP|";
    return;
end
phi = atan2(B,A);
delta = asin(P.yP/Rn);
for q1 = [delta-phi, pi-delta-phi]
    s1 = sin(q1); c1 = cos(q1);
    s5 = -(s1*r13+c1*r23);
    if abs(s5)>1+1e-10, continue; end
    s5 = max(-1,min(1,s5));
    c5m = sqrt(max(0,1-s5^2));
    if c5m<1e-10, continue; end
    for c5_sign = [1,-1]
        c5 = c5_sign*c5m;
        P_ = c1*r13-s1*r23;
        Q_ = r33;
        U = c1*p(1)-s1*p(2);
        X = U-P.d5*P_+P.yW*Q_/c5;
        Y = p(3)-P.d1-P.d5*Q_-P.yW*P_/c5-P.zA;

        s3 = (X^2+Y^2-P.a2^2-P.a3^2)/(2*P.a2*P.a3);
        if abs(s3)>1+1e-10, continue; end
        s3 = max(-1,min(1,s3));
        c3m = sqrt(max(0,1-s3^2));
        for c3_sign = [1,-1]
            c3 = c3_sign*c3m;
            q3 = atan2(s3,c3)-P.q3Offset;
            radiusSquared = X^2+Y^2;
            if radiusSquared<1e-12, continue; end
            c2 = (-P.a3*c3*X+(P.a2+P.a3*s3)*Y)/radiusSquared;
            s2 = (-P.a3*c3*Y-(P.a2+P.a3*s3)*X)/radiusSquared;
            q2 = atan2(s2,c2);
            th = q3-q2;
            c4 = (-P_*cos(th)+Q_*sin(th))/c5;
            s4 = (-P_*sin(th)-Q_*cos(th))/c5;
            q4 = atan2(s4,c4);
            Rz_mth = [cos(th) sin(th) 0; -sin(th) cos(th) 0; 0 0 1];
            Rx_p2 = [1 0 0; 0 0 -1; 0 1 0];
            Rz_q1 = [c1 -s1 0; s1 c1 0; 0 0 1];
            R36 = Rz_mth*Rx_p2*Rz_q1*R;
            q6 = atan2(-R36(3,2)/c5,R36(3,1)/c5);
            candidate = [q1 q2 q3 q4 atan2(s5,c5) q6];
            candidate = atan2(sin(candidate),cos(candidate));
            if any(candidate<P.qmin | candidate>P.qmax), continue; end
            difference = atan2(sin(sols-candidate),cos(sols-candidate));
            if isempty(sols) || ~any(vecnorm(difference,2,2)<1e-7)
                sols(end+1,:) = candidate; %#ok<AGROW>
            end
        end
    end
end
if ~isempty(sols), info.reason = "ok"; end
end

function validateTarget(T)
if ~isequal(size(T),[4 4]) || any(~isfinite(T(:))) || ...
        norm(T(4,:)-[0 0 0 1]) > 1e-10 || ...
        norm(T(1:3,1:3).'*T(1:3,1:3)-eye(3),'fro') > 1e-8 || ...
        det(T(1:3,1:3)) < 0
    error('robot_ver2:InvalidTargetPose', ...
        'Target must be a finite, proper 4-by-4 rigid transform.');
end
end

function value = option(options,name,defaultValue)
if isfield(options,name) && ~isempty(options.(name))
    value = options.(name);
else
    value = defaultValue;
end
end

function [q,pError,rError] = validatedCandidates( ...
        candidates,robot,target,limits,pTolerance,rTolerance)
q = zeros(0,robot.structure.dof);
pError = zeros(0,1);
rError = zeros(0,1);
for i = 1:size(candidates,1)
    candidate = candidates(i,:);
    if any(candidate < limits(:,1).'-1e-9 | candidate > limits(:,2).'+1e-9)
        continue;
    end
    achieved = getTransform(robot.model,candidate, ...
        char(robot.structure.frames.endEffector));
    pe = norm(achieved(1:3,4)-target(1:3,4));
    relativeRotation = target(1:3,1:3).'*achieved(1:3,1:3);
    re = acos(max(-1,min(1,(trace(relativeRotation)-1)/2)));
    if pe > pTolerance || re > rTolerance
        continue;
    end
    wrappedDifference = atan2(sin(q-candidate),cos(q-candidate));
    if ~isempty(q) && any(vecnorm(wrappedDifference,2,2) < 1e-6)
        continue;
    end
    q(end+1,:) = candidate; %#ok<AGROW>
    pError(end+1,1) = pe; %#ok<AGROW>
    rError(end+1,1) = re; %#ok<AGROW>
end
end
