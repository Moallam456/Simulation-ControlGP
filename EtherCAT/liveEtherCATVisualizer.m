%% LIVE ETHERCAT VISUALIZER
%
% Receives the six A6-EC position feedback values from
% the FreeRTOS/SOEM EtherCAT test over UDP and displays
% them using the project's UR5 visual mesh model.

clear;
clc;
close all;


%% ========================================================================
%  1. ADD PROJECT ROOT TO MATLAB PATH
% =========================================================================

thisFileFolder = fileparts(mfilename("fullpath"));

repoRoot = fileparts(thisFileFolder);

addpath(repoRoot);

fprintf("Repository root:\n%s\n\n", repoRoot);


%% ========================================================================
%  2. UDP RECEIVER
% =========================================================================

PORT = 5005;

udp = udpport( ...
    "datagram", ...
    "IPV4", ...
    "LocalPort", PORT);

fprintf( ...
    "Listening for EtherCAT telemetry on UDP port %d...\n", ...
    PORT);


%% ========================================================================
%  3. LOAD PROJECT ROBOT CONFIGURATION
% =========================================================================

robot = config.UR5();


%% ========================================================================
%  4. BUILD UR5 VISUAL ROBOT
% =========================================================================
%
% Uses the UR5 visual meshes stored in the repository.

visualRobot = robotmodel.buildUR5geometry(robot);


%% ========================================================================
%  5. INITIAL JOINT CONFIGURATION
% =========================================================================
%
% rigidBodyTree uses:
%
%       DataFormat = "column"
%
% therefore q must be:
%
%       6 x 1

q = zeros(robot.dof, 1);


%% ========================================================================
%  6. CREATE 3-D FIGURE
% =========================================================================

fig = figure( ...
    "Name", "Live EtherCAT Robot", ...
    "NumberTitle", "off", ...
    "Color", [0.12 0.12 0.12], ...
    "Renderer", "opengl");

ax = axes( ...
    "Parent", fig);

hold(ax, "on");

grid(ax, "on");

axis(ax, "equal");

% Keep the physical XYZ proportions fixed when rotating the camera.
axis(ax, "vis3d");


%% ------------------------------------------------------------------------
% 3-D CAMERA
% -------------------------------------------------------------------------

% Perspective gives stronger depth perception than MATLAB's default
% orthographic robot view.
ax.Projection = "perspective";

% Initial viewing angle.
view(ax, 135, 25);


%% ------------------------------------------------------------------------
% AXES APPEARANCE
% -------------------------------------------------------------------------

ax.Color = [0.12 0.12 0.12];

ax.XColor = [0.85 0.85 0.85];
ax.YColor = [0.85 0.85 0.85];
ax.ZColor = [0.85 0.85 0.85];

ax.GridColor = [0.65 0.65 0.65];
ax.GridAlpha = 0.30;

xlabel(ax, "X [m]");
ylabel(ax, "Y [m]");
zlabel(ax, "Z [m]");


%% ------------------------------------------------------------------------
% ALLOW INTERACTIVE ROTATION
% -------------------------------------------------------------------------

rotate3d(fig, "on");


%% ========================================================================
%  7. INITIAL ROBOT DRAW
% =========================================================================

show( ...
    visualRobot, ...
    q, ...
    "Parent", ax, ...
    "Frames", "off", ...
    "Visuals", "on", ...
    "Collisions", "off", ...
    "PreservePlot", false);


title( ...
    ax, ...
    "Waiting for EtherCAT feedback...", ...
    "Color", [0.95 0.95 0.95]);


%% ========================================================================
%  8. IMPROVE 3-D MESH RENDERING
% =========================================================================
%
% MATLAB's default mesh rendering can make the robot look like a
% flat gray silhouette.
%
% We:
%
%   1. remove mesh triangle edges
%   2. enable Gouraud surface lighting
%   3. add multiple light sources
%

meshObjects = findobj(ax, "Type", "Patch");

for k = 1:numel(meshObjects)

    meshObjects(k).EdgeColor = "none";

    meshObjects(k).FaceLighting = "gouraud";

end


%% ------------------------------------------------------------------------
% LIGHTING
% -------------------------------------------------------------------------

% Light coming from the camera.
camlight(ax, "headlight");

% Additional side light makes cylindrical / curved surfaces much easier
% to perceive in 3-D.
camlight(ax, "right");

lighting(ax, "gouraud");


%% ------------------------------------------------------------------------
% MATERIAL
% -------------------------------------------------------------------------
%
% Changes how strongly surfaces respond to the light.

material(ax, "dull");


%% ------------------------------------------------------------------------
% CAMERA / AXIS LIMITS
% -------------------------------------------------------------------------
%
% Give some space around the UR5 so perspective is visible and MATLAB
% does not constantly resize the scene.

xlim(ax, [-1.0  0.4]);
ylim(ax, [-0.8  0.8]);
zlim(ax, [-0.5  1.1]);


drawnow;


%% ========================================================================
%  9. A6 POSITION SCALE
% =========================================================================
%
% A6 servo:
%
%       17-bit encoder
%
% therefore:
%
%       2^17 = 131072 counts/revolution
%
% Current electronic gear ratio:
%
%       1 : 1
%
% Therefore:
%
%             2*pi
%       q = -------- * position
%            131072
%

COUNTS_PER_REV = 131072;

countsToRadians = ...
    2*pi / COUNTS_PER_REV;


%% ========================================================================
%  10. LIVE TELEMETRY LOOP
% =========================================================================

fprintf("\n");
fprintf("========================================\n");
fprintf("LIVE ETHERCAT VISUALIZATION\n");
fprintf("========================================\n");
fprintf("Waiting for six joint feedback values...\n");
fprintf("Close the figure to stop visualization.\n");
fprintf("========================================\n\n");


while isvalid(fig)

    %% --------------------------------------------------------------------
    % CHECK WHETHER UDP PACKETS ARE AVAILABLE
    % ---------------------------------------------------------------------

    if udp.NumDatagramsAvailable > 0

        % Read every packet currently waiting.
        %
        % Visualization only needs the newest packet.

        packets = read( ...
            udp, ...
            udp.NumDatagramsAvailable, ...
            "uint8");


        %% ----------------------------------------------------------------
        % USE NEWEST PACKET
        % -----------------------------------------------------------------

        bytes = uint8( ...
            packets(end).Data);


        % C sends:
        %
        %       int32_t actual_positions[6]
        %
        % Each int32 = 4 bytes.
        %
        % Therefore:
        %
        %       6 * 4 = 24 bytes

        if numel(bytes) ~= 24

            fprintf( ...
                "Unexpected packet size: %d bytes\n", ...
                numel(bytes));

            continue;

        end


        %% ----------------------------------------------------------------
        % RAW BYTES -> SIX int32 VALUES
        % -----------------------------------------------------------------

        jointCounts = typecast( ...
            bytes(:).', ...
            "int32");


        jointCounts = double( ...
            jointCounts(:));


        %% ----------------------------------------------------------------
        % ENCODER COUNTS -> JOINT ANGLES [rad]
        % -----------------------------------------------------------------

        q = ...
            jointCounts ...
            * countsToRadians;


        %% ----------------------------------------------------------------
        % UPDATE ROBOT
        % -----------------------------------------------------------------

        show( ...
            visualRobot, ...
            q, ...
            "Parent", ax, ...
            "Frames", "off", ...
            "Visuals", "on", ...
            "Collisions", "off", ...
            "FastUpdate", true, ...
            "PreservePlot", false);


        %% ----------------------------------------------------------------
        % JOINT ANGLES FOR DISPLAY
        % -----------------------------------------------------------------

        qDeg = rad2deg(q);


        titleText = sprintf( ...
            ['LIVE ETHERCAT FEEDBACK\n' ...
             'J1 %.2f°   J2 %.2f°   J3 %.2f°   ' ...
             'J4 %.2f°   J5 %.2f°   J6 %.2f°'], ...
            qDeg(1), ...
            qDeg(2), ...
            qDeg(3), ...
            qDeg(4), ...
            qDeg(5), ...
            qDeg(6));


        title( ...
            ax, ...
            titleText, ...
            "Color", [0.95 0.95 0.95]);


        %% ----------------------------------------------------------------
        % CONSOLE FEEDBACK
        % -----------------------------------------------------------------

        fprintf( ...
            ['J1=%7.2f  J2=%7.2f  J3=%7.2f  ' ...
             'J4=%7.2f  J5=%7.2f  J6=%7.2f deg\n'], ...
            qDeg(1), ...
            qDeg(2), ...
            qDeg(3), ...
            qDeg(4), ...
            qDeg(5), ...
            qDeg(6));


        %% ----------------------------------------------------------------
        % DRAW
        % -----------------------------------------------------------------

        drawnow limitrate;


    else

        % Prevent MATLAB from using an entire CPU core while waiting.

        pause(0.005);

    end

end


%% ========================================================================
%  11. CLEANUP
% =========================================================================

clear udp;

fprintf("\nVisualizer closed.\n");