function [Q, info] = inverse_kinematics(T06, robot)

% ==========================================================
% INVERSE KINEMATICS
% ==========================================================
%
% Analytical IK for the ARC Mate 100iD style robot.
%
% INPUTS:
%   T06     Desired TCP pose (4x4)
%   robot   Robot struct from robot_params()
%
% OUTPUTS:
%   Q       Nx6 matrix of IK solutions [rad]
%   info    Solver information
%
% ==========================================================

%% Geometry

g = robot.geometry;

a1 = g.a1;
a2 = g.a2;
a3 = g.a3;

d1 = g.d1;
d4 = g.d4;
d6 = g.dTool;

%% Extract TCP Pose

R06 = T06(1:3,1:3);
p06 = T06(1:3,4);

%% Wrist Center

pW = p06 + d6 * R06(:,2);

xW = pW(1);
yW = pW(2);
zW = pW(3);

%% Solution Storage

solutions = [];

%% ---------------------------------------------------------
%% Theta 3
%% ---------------------------------------------------------

s3 = (zW - d1) / a3;

if abs(s3) > 1 + 1e-9

    Q = zeros(0,6);

    info.success = false;
    info.numSolutions = 0;
    info.message = ...
        "Target outside workspace.";

    return;

end

s3 = max(-1,min(1,s3));

c3_abs = sqrt(max(0,1 - s3^2));

%% =========================================================
%% Elbow Configurations
%% =========================================================

for elbow = 1:2

    c3 = (-1)^(elbow-1) * c3_abs;

    theta3 = atan2(s3,c3);

    A = a2 + a3*c3;
    B = d4;

    rho2 = A^2 + B^2;

    if rho2 < 1e-12
        continue;
    end

    rho = sqrt(rho2);

    %% -----------------------------------------------------
    %% Theta 2
    %% -----------------------------------------------------

    X = xW^2 + yW^2 - a1^2 - rho2;

    cVal = X/(2*a1*rho);

    if abs(cVal) > 1 + 1e-9
        continue;
    end

    cVal = max(-1,min(1,cVal));

    alpha = acos(cVal);

    psi = atan2(B,A);

    %% Shoulder Configurations

    for shoulder = 1:2

        if shoulder == 1

            theta2 = psi - alpha;

        else

            theta2 = psi + alpha;

        end

        c2 = cos(theta2);
        s2 = sin(theta2);

        %% -------------------------------------------------
        %% Theta 1
        %% -------------------------------------------------

        P = A*c2 + B*s2;
        Qp = -A*s2 + B*c2;

        theta1 = ...
            atan2(yW,xW) + atan2(Qp,a1 + P);

        %% -------------------------------------------------
        %% Rotation Base -> Wrist
        %% -------------------------------------------------

        c12 = cos(theta1 + theta2);
        s12 = sin(theta1 + theta2);

        R03 = [ ...
            c12*c3  -c12*s3   s12;
            s12*c3  -s12*s3  -c12;
            s3        c3       0 ];

        R36 = R03' * R06;

        %% -------------------------------------------------
        %% Wrist Solution
        %% -------------------------------------------------

        c5 = -R36(3,2);

        if abs(c5) > 1 + 1e-9
            continue;
        end

        c5 = max(-1,min(1,c5));

        s5_abs = sqrt(max(0,1-c5^2));

        for wrist = 1