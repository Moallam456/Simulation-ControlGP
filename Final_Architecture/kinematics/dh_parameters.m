function dh = dh_parameters(robot)

% ==========================================================
% DH PARAMETERS
% ==========================================================
% FANUC ARC Mate 100iD
%
% Generated automatically from robot_params.
% ==========================================================

g = robot.geometry;

dh.a = [ ...
    g.a1;
    g.a2;
    g.a3;
    0;
    0;
    0 ];

dh.alpha = [ ...
     0;
     pi/2;
     0;
    -pi/2;
     pi/2;
    -pi/2 ];

dh.d = [ ...
    g.d1;
    0;
    0;
    g.d4;
    0;
    g.dTool ];

dh.thetaOffset = [ ...
     0;
     0;
     pi;
    -pi/2;
     0;
     0 ];

end