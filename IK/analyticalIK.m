function [Q, info] = fanuc_arc_mate_100id_ik(T06, robot)
% Analytical IK for FANUC ARC Mate 100iD matching the DH derived from
% fanuc_arc_mate_100id_frames():
%
%   i |  a_i  | alpha_i |  d_i  | theta_offset
%   --+-------+---------+-------+-------------
%   1 | 0.150 |    0    | 0.525 |      0
%   2 | 0.770 |  +pi/2  |   0   |      0
%   3 | 0.100 |    0    |   0   |     pi
%   4 |   0   |  -pi/2  | 0.740 |   -pi/2
%   5 |   0   |  +pi/2  |   0   |      0
%   6 |   0   |  -pi/2  | dTool |      0
%
% INPUT
%   T06   : 4x4 desired TCP pose in Frame 0
%   robot : struct with a1, a2, a3, d1, dTool, d4
%
% OUTPUT
%   Q     : N x 6 joint solutions [rad]
%   info  : .success (logical), .numSolutions (int)

% ---- link lengths (from robot struct) -----------------------------------
a1 = robot.a1;    a2 = robot.a2;    a3 = robot.a3;
d1 = robot.d1;    d6 = robot.dTool;
if isfield(robot,'d4'), d4 = robot.d4; else, d4 = 0.740; end

% ---- extract TCP and wrist centre ---------------------------------------
R06 = T06(1:3,1:3);
p06 = T06(1:3,4);
pW  = p06 + d6 * R06(:,2);        % origin of frame 4 (wrist centre)
xW  = pW(1);  yW = pW(2);  zW = pW(3);

sols = zeros(0,6);

% =========================================================================
%  THETA 3  --  z_W = d1 + a3*sin(theta3)
% =========================================================================
s_th3 = (zW - d1) / a3;
if abs(s_th3) > 1 + 1e-9
    Q = zeros(0,6);  info.success = false;  info.numSolutions = 0;
    return;
end
s_th3  = max(-1, min(1, s_th3));
c3_abs = sqrt(max(0, 1 - s_th3^2));

if abs(a1) < 1e-9
    warning('fanuc_arc_mate_100id_ik: a1=0 case not handled.');
    Q = zeros(0,6);  info.success = false;  info.numSolutions = 0;
    return;
end

% =========================================================================
%  Elbow loop
% =========================================================================
for iElb = 1:2
    c_th3 = (-1)^(iElb-1) * c3_abs;
    th3   = atan2(s_th3, c_th3);

    A = a2 + a3 * c_th3;
    B = d4;
    rho2 = A^2 + B^2;
    if rho2 < 1e-12, continue; end
    rho = sqrt(rho2);

    % ---- THETA 2 ----
    X  = xW^2 + yW^2 - a1^2 - rho2;
    cv = X / (2 * a1 * rho);
    if abs(cv) > 1 + 1e-9, continue; end
    cv    = max(-1, min(1, cv));
    alpha = acos(cv);
    psi   = atan2(B, A);

    for iSh = 1:2
        if iSh == 1
            th2 = psi - alpha;
        else
            th2 = psi + alpha;
        end
        c2 = cos(th2);  s2 = sin(th2);

        % ---- THETA 1 ----
        P  =  A*c2 + B*s2;
        Qp = -A*s2 + B*c2;
        th1 = atan2(yW, xW) + atan2(Qp, a1 + P);

        c12 = cos(th1 + th2);  s12 = sin(th1 + th2);

        % ---- R_03 ----
        R03 = [ c12*c_th3,  -c12*s_th3,   s12;
                s12*c_th3,  -s12*s_th3,  -c12;
                s_th3,       c_th3,       0  ];
        R36 = R03.' * R06;

        % ---- WRIST ----
        cv5 = -R36(3,2);
        if abs(cv5) > 1 + 1e-9, continue; end
        cv5 = max(-1, min(1, cv5));
        sv5 = sqrt(max(0, 1 - cv5^2));

        for iWr = 1:2
            s5 = (-1)^(iWr-1) * sv5;
            q5 = atan2(s5, cv5);

            if sv5 < 1e-8
                q4 = 0;
                q6 = atan2(R36(1,1), R36(1,3));
            else
                q4 = atan2(-R36(1,2)/s5,  R36(2,2)/s5);
                q6 = atan2( R36(3,3)/s5, -R36(3,1)/s5);
            end

            q1 = th1;
            q2 = th2;
            q3 = th3 - pi;                 % theta3 = q3 + pi

            sols(end+1,:) = [q1, q2, q3, q4, q5, q6]; %#ok<AGROW>
        end
    end
end

Q = sols;
info.success      = ~isempty(Q);
info.numSolutions = size(Q,1);
end
