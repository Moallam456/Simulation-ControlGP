function [q, allSols, info] = ikSolver(T_target, q_seed, P)
%IKSOLVER  Closed-form branches for the nominal ver2 URDF geometry.
%
%   q        : 1x6 joint solution closest to q_seed  ([] if unreachable)
%   allSols  : Nx6 algebraic candidates (validate with the robot's FK)
%   info     : diagnostic struct
%
%   T_target : 4x4 desired TCP pose in the BASE frame.
%   q_seed   : 1x6 initial guess (used only to pick the branch).
%   P        : parameter struct from robotParams().
%
%   The Joint_6 URDF frame rounds a right angle to 1.5708 radians. Validate
%   branches against the imported model before commanding robot motion.

if nargin < 3 || isempty(P), P = robotParams(); end
if nargin < 2 || isempty(q_seed), q_seed = zeros(1,6); end
q_seed = reshape(q_seed,1,[]);
if numel(q_seed) ~= 6 || any(~isfinite(q_seed))
    error('RecentIK:InvalidSeed','q_seed must contain six finite angles.');
end
if ~isRigidPose(T_target) || ~isRigidPose(P.T_flange_tcp)
    error('RecentIK:InvalidPose', ...
        'Target and flange-to-TCP transforms must be proper 4-by-4 poses.');
end
T_target = T_target / P.T_flange_tcp;

% ---------------- unpack target ----------------
p   = T_target(1:3,4);
R   = T_target(1:3,1:3);
r13 = R(1,3);  r23 = R(2,3);  r33 = R(3,3);

% ---------------- constants ----------------
d1 = P.d1; yP = P.yP; zA = P.zA;
a2 = P.a2; a3 = P.a3;
d5 = P.d5; yW = P.yW;

sols = zeros(0,6);
info.reason = "";

% ---------------- Step 1: q1 ----------------
A  = p(1) - d5*r13;
B  = p(2) - d5*r23;
Rn = hypot(A, B);
if Rn < abs(yP)
    q = []; allSols = sols; info.reason = "unreachable: R_xy < |yP|"; return;
end
phi   = atan2(B, A);
delta = asin(yP / Rn);
q1_list = [delta - phi, pi - delta - phi];

for q1 = q1_list
    s1 = sin(q1); c1 = cos(q1);

    % ---------------- Step 2: q5 ----------------
    s5 = -(s1*r13 + c1*r23);
    if abs(s5) > 1 + 1e-10, continue; end
    s5 = max(-1,min(1,s5));
    c5m = sqrt(max(0, 1 - s5^2));
    if c5m < 1e-10, continue; end

    for c5_sign = [1, -1]
        c5 = c5_sign * c5m;

        % ---------------- Step 3: wrist direction ----------------
        P_ = c1*r13 - s1*r23;
        Q_ = r33;

        % ---------------- Step 4: reduced arm position ----------------
        U  = c1*p(1) - s1*p(2);
        px = U - d5*P_ + yW*Q_/c5;             % +yW = -0.175
        pz = p(3) - d1 - d5*Q_ - yW*P_/c5;
        X  = px;
        Y  = pz - zA;

        % ---------------- Step 5: q3 ----------------
        s3 = (X^2 + Y^2 - a2^2 - a3^2) / (2*a2*a3);
        if abs(s3) > 1 + 1e-10, continue; end
        s3 = max(-1,min(1,s3));
        c3m = sqrt(max(0, 1 - s3^2));

        for c3_sign = [1, -1]
            c3 = c3_sign * c3m;
            q3Planar = atan2(s3, c3);
            q3 = q3Planar - P.q3Offset;

            % ---------------- Step 6: q2 ----------------
            det = X^2 + Y^2;
            if det < 1e-12, continue; end
            c2 = (-a3*c3*X + (a2 + a3*s3)*Y) / det;
            s2 = (-a3*c3*Y - (a2 + a3*s3)*X) / det;
            q2 = atan2(s2, c2);

            % ---------------- Step 7: q4 ----------------
            th = q3 - q2;
            c4 = (-P_*cos(th) + Q_*sin(th)) / c5;
            s4 = (-P_*sin(th) - Q_*cos(th)) / c5;
            q4 = atan2(s4, c4);

            % ---------------- Step 8: q6 ----------------
            % R_36 = Rz(-th) * Rx(pi/2) * Rz(q1) * R
            Rz_mth  = [cos(th)  sin(th) 0; -sin(th) cos(th) 0; 0 0 1];
            Rx_p2   = [1 0 0; 0 0 -1; 0 1 0];
            Rz_q1   = [cos(q1) -sin(q1) 0; sin(q1) cos(q1) 0; 0 0 1];
            R36     = Rz_mth * Rx_p2 * Rz_q1 * R;
            q6 = atan2(-R36(3,2)/c5, R36(3,1)/c5);

            q_cand = [q1 q2 q3 q4 atan2(s5,c5) q6];

            % wrap and enforce limits
            q_cand = atan2(sin(q_cand), cos(q_cand));
            if any(q_cand < P.qmin | q_cand > P.qmax), continue; end
            difference = atan2(sin(sols-q_cand),cos(sols-q_cand));
            if isempty(sols) || ~any(vecnorm(difference,2,2) < 1e-7)
                sols(end+1, :) = q_cand; %#ok<AGROW>
            end
        end
    end
end

allSols = sols;
if isempty(sols)
    q = []; info.reason = "no closed-form branch found in joint limits"; return;
end

% ---------------- pick branch closest to seed ----------------
d = sols - q_seed;
d = atan2(sin(d), cos(d));
[~, idx] = min(sum(d.^2, 2));
q = sols(idx, :);
info.reason = "ok";
end

function valid = isRigidPose(T)
valid = isequal(size(T),[4 4]) && all(isfinite(T(:)));
if valid
    R = T(1:3,1:3);
    valid = norm(T(4,:)-[0 0 0 1]) < 1e-10 && ...
        norm(R.'*R-eye(3),'fro') < 1e-8 && det(R) > 0;
end
end
