function P = robotParams()
%ROBOTPARAMS  All geometry constants for URDF_Assembly_Robot_ver2.
%   Nominal IK geometry derived from the robot_ver2 URDF. The imported URDF
%   remains authoritative for FK; testIK checks these constants against it.
%   Values are in metres and radians.

P.name = "URDF_Assembly_Robot_ver2";
P.n    = 6;

% ---- Arm (first 3 joints) ----------------------------------------------
P.d1  = 0.2000;      % base height (Joint_1 origin z)
P.yP  = -0.2425;     % constant y of wrist point in Link_1
P.zA  = 0.1320;      % z-offset from Link_1 origin to Joint_2
P.a2  = 0.589954679438418; % J2 -> J3 (planar)
P.a3x = 0.404499960921119; % negative x of J3 -> J4
P.a3y = -4.53236012523117e-5; % y of J3 -> J4
P.a3  = hypot(P.a3x,P.a3y);
P.q3Offset = atan2(-P.a3y,P.a3x);

% ---- Wrist (last 3 joints) ---------------------------------------------
P.d5  = 0.0555;      % J6 x offset in Link_5
P.yW  = -0.0554999969603261 - 0.1195;

% ---- Joint limits (provisional in the URDF too) ------------------------
P.qmin = [-pi -pi -pi -pi -pi -pi];
P.qmax = [ pi  pi  pi  pi  pi  pi];

% ---- TCP --------------------------------------------------------------
P.T_flange_tcp = eye(4);    % identity by default
end
