clear
clc
close all

%% ========================================================================
%  MECHANICAL TEAM - AUTOMATIC SIMULATION HANDOFF
% =========================================================================
%
% This script evaluates the current welding-robot geometry candidates and
% automatically exports:
%
%   1. Link-length comparison table (.xlsx)
%   2. Link-length comparison table (.csv)
%   3. MATLAB results (.mat)
%   4. Radial workspace plots
%   5. Maximum-reach robot poses
%   6. Robot + welding-station visualizations
%   7. Summary comparison charts
%
% IMPORTANT:
%
% These are PRELIMINARY SIMULATION RESULTS.
%
% They currently use:
%
%   - UR5-scale preliminary wrist/base dimensions
%   - simplified collision geometry
%   - preliminary TIG-tool geometry
%   - preliminary welding-table geometry
%
% They are NOT final mechanical dimensions.


%% ========================================================================
% 1. OUTPUT DIRECTORY
% =========================================================================

projectRoot = fileparts(fileparts(mfilename("fullpath")));

outputFolder = fullfile( ...
    projectRoot, ...
    "results", ...
    "mechanicalHandoff");

plotFolder = fullfile( ...
    outputFolder, ...
    "plots");

poseFolder = fullfile( ...
    outputFolder, ...
    "robotPoses");


if ~exist(outputFolder,"dir")
    mkdir(outputFolder);
end

if ~exist(plotFolder,"dir")
    mkdir(plotFolder);
end

if ~exist(poseFolder,"dir")
    mkdir(poseFolder);
end


fprintf("\n============================================================\n");
fprintf("MECHANICAL HANDOFF EXPORT\n");
fprintf("============================================================\n");

fprintf("\nOutput folder:\n%s\n\n",outputFolder);


%% ========================================================================
% 2. CANDIDATE LINK LENGTHS
% =========================================================================
%
% Candidate matrix:
%
%       [L2   L3]
%
% Units: metres

candidates = [

    0.400   0.350
    0.400   0.400
    0.400   0.450
    0.350   0.300
    0.375   0.375
    0.425   0.375
    0.450   0.350
    0.375   0.425
    0.350   0.450

];

numCandidates = size(candidates,1);


%% ========================================================================
% 3. PRELIMINARY FIXED GEOMETRY
% =========================================================================
%
% These dimensions currently remain fixed during the L2/L3 comparison.

fixedGeometry.d1 = 0.089159;

fixedGeometry.d4 = 0.10915;
fixedGeometry.d5 = 0.09465;
fixedGeometry.d6 = 0.0823;


%% ========================================================================
% 4. SAMPLING SETTINGS
% =========================================================================
%
% Use the same random seed for every geometry so that the comparison is
% repeatable.
%
% 10,000 samples is suitable for the current preliminary handoff.
%
% Increase to 50,000 later for shortlisted geometries.

numSamples = 10000;

randomSeed = 1;


%% ========================================================================
% 5. CREATE WELDING STATION
% =========================================================================

station = ...
    environment.buildPreliminaryWeldingStation();


%% ========================================================================
% 6. PREALLOCATE RESULT ARRAYS
% =========================================================================

candidateID = (1:numCandidates).';

L2_mm = zeros(numCandidates,1);

L3_mm = zeros(numCandidates,1);

totalMainLink_mm = zeros(numCandidates,1);

linkRatio = zeros(numCandidates,1);


kinematicMaxReach_mm = zeros(numCandidates,1);

collisionFreeMaxReach_mm = zeros(numCandidates,1);

environmentMaxReach_mm = zeros(numCandidates,1);


minHorizontalRadius_mm = zeros(numCandidates,1);


xSpan_mm = zeros(numCandidates,1);

ySpan_mm = zeros(numCandidates,1);

zSpan_mm = zeros(numCandidates,1);


collisionFreePercent = zeros(numCandidates,1);

environmentValidPercent = zeros(numCandidates,1);

selfCollisionPercent = zeros(numCandidates,1);

worldCollisionPercent = zeros(numCandidates,1);


allResults = cell(numCandidates,1);


%% ========================================================================
% 7. RUN ALL CANDIDATES
% =========================================================================

for candidate = 1:numCandidates

    fprintf("\n============================================================\n");

    fprintf( ...
        "Running Candidate %d / %d\n", ...
        candidate, ...
        numCandidates);

    fprintf("============================================================\n");


    %% --------------------------------------------------------------------
    % Geometry
    % ---------------------------------------------------------------------

    geometry = fixedGeometry;

    geometry.L2 = candidates(candidate,1);

    geometry.L3 = candidates(candidate,2);


    fprintf( ...
        "L2 = %.0f mm, L3 = %.0f mm\n", ...
        geometry.L2*1000, ...
        geometry.L3*1000);


    %% --------------------------------------------------------------------
    % Build robot
    % ---------------------------------------------------------------------

    robot = ...
        config.WeldingRobot(geometry);


    robotModel = ...
        robotmodel.buildRobot(robot);


    collisionRobot = ...
        robotmodel.addPreliminaryCollisionGeometry( ...
            robotModel, ...
            robot);


    %% --------------------------------------------------------------------
    % Kinematic workspace
    % ---------------------------------------------------------------------

    fprintf("  Calculating kinematic workspace...\n");

    kinematicWorkspace = ...
        analysis.workspace( ...
            robotModel, ...
            robot, ...
            numSamples, ...
            randomSeed);


    %% --------------------------------------------------------------------
    % Self-collision-free workspace
    % ---------------------------------------------------------------------

    fprintf("  Calculating self-collision-free workspace...\n");

    collisionWorkspace = ...
        analysis.collisionFreeWorkspace( ...
            collisionRobot, ...
            robot, ...
            numSamples, ...
            randomSeed);


    %% --------------------------------------------------------------------
    % Environment-constrained workspace
    % ---------------------------------------------------------------------

    fprintf("  Calculating environment-constrained workspace...\n");

    environmentWorkspace = ...
        analysis.environmentConstrainedWorkspace( ...
            collisionRobot, ...
            robot, ...
            station.worldObjects, ...
            numSamples, ...
            randomSeed);


    %% --------------------------------------------------------------------
    % Store geometry
    % ---------------------------------------------------------------------

    L2_mm(candidate) = ...
        geometry.L2*1000;

    L3_mm(candidate) = ...
        geometry.L3*1000;

    totalMainLink_mm(candidate) = ...
        (geometry.L2 + geometry.L3)*1000;

    linkRatio(candidate) = ...
        geometry.L2 / geometry.L3;


    %% --------------------------------------------------------------------
    % Reach values
    % ---------------------------------------------------------------------

    kinematicMaxReach_mm(candidate) = ...
        kinematicWorkspace.maxReach*1000;

    collisionFreeMaxReach_mm(candidate) = ...
        collisionWorkspace.maxReach*1000;

    environmentMaxReach_mm(candidate) = ...
        environmentWorkspace.maxReach*1000;


    minHorizontalRadius_mm(candidate) = ...
        environmentWorkspace.minHorizontalReach*1000;


    %% --------------------------------------------------------------------
    % Environment-constrained Cartesian spans
    % ---------------------------------------------------------------------

    xSpan_mm(candidate) = ...
        diff(environmentWorkspace.xRange)*1000;

    ySpan_mm(candidate) = ...
        diff(environmentWorkspace.yRange)*1000;

    zSpan_mm(candidate) = ...
        diff(environmentWorkspace.zRange)*1000;


    %% --------------------------------------------------------------------
    % Collision statistics
    % ---------------------------------------------------------------------

    collisionFreePercent(candidate) = ...
        collisionWorkspace.collisionFreePercent;

    environmentValidPercent(candidate) = ...
        environmentWorkspace.validPercent;

    selfCollisionPercent(candidate) = ...
        environmentWorkspace.selfCollisionPercent;

    worldCollisionPercent(candidate) = ...
        environmentWorkspace.worldCollisionPercent;


    %% --------------------------------------------------------------------
    % Save complete candidate structures
    % ---------------------------------------------------------------------

    candidateResult.geometry = geometry;

    candidateResult.robot = robot;

    candidateResult.kinematicWorkspace = ...
        kinematicWorkspace;

    candidateResult.collisionWorkspace = ...
        collisionWorkspace;

    candidateResult.environmentWorkspace = ...
        environmentWorkspace;

    allResults{candidate} = candidateResult;


    %% ====================================================================
    % 8. SAVE RADIAL WORKSPACE PLOT
    % ====================================================================

    fig = figure( ...
        "Visible","off");


    scatter( ...
        environmentWorkspace.horizontalRadius, ...
        environmentWorkspace.points(:,3), ...
        5, ...
        ".");


    xlabel("Horizontal Radius \rho [m]");

    ylabel("Z [m]");


    title(sprintf( ...
        "Environment-Constrained Workspace - L2 %.0f mm, L3 %.0f mm", ...
        geometry.L2*1000, ...
        geometry.L3*1000));


    grid on


    filename = sprintf( ...
        "Candidate_%02d_L2_%.0f_L3_%.0f_RadialWorkspace.png", ...
        candidate, ...
        geometry.L2*1000, ...
        geometry.L3*1000);


    exportgraphics( ...
        fig, ...
        fullfile(plotFolder,filename), ...
        "Resolution",300);


    close(fig);


    %% ====================================================================
    % 9. FIND MAXIMUM VALID REACH CONFIGURATION
    % ====================================================================

    [~,maxReachIndex] = ...
        max(environmentWorkspace.radius3D);


    qMaxReach = ...
        environmentWorkspace.q(:,maxReachIndex);


    %% ====================================================================
    % 10. SAVE ROBOT MAXIMUM-REACH POSE
    % ====================================================================

    fig = figure( ...
        "Visible","off");


    ax = show( ...
        collisionRobot, ...
        qMaxReach, ...
        "Visuals","off", ...
        "Collisions","on", ...
        "Frames","off");


    hold(ax,"on");


    %% Welding table

    [~,tablePatch] = ...
        show( ...
            station.table.object, ...
            "Parent",ax);

    tablePatch.FaceAlpha = 0.35;


    %% Mounting plane

    [~,mountingPatch] = ...
        show( ...
            station.mountingPlane.object, ...
            "Parent",ax);

    mountingPatch.FaceAlpha = 0.15;


    xlabel(ax,"X [m]");
    ylabel(ax,"Y [m]");
    zlabel(ax,"Z [m]");


    title(ax,sprintf( ...
        "Candidate %d - Maximum Valid Reach %.0f mm", ...
        candidate, ...
        environmentWorkspace.maxReach*1000));


    axis(ax,"equal");

    grid(ax,"on");

    view(ax,135,25);


    filename = sprintf( ...
        "Candidate_%02d_L2_%.0f_L3_%.0f_MaxReachPose.png", ...
        candidate, ...
        geometry.L2*1000, ...
        geometry.L3*1000);


    exportgraphics( ...
        fig, ...
        fullfile(poseFolder,filename), ...
        "Resolution",300);


    close(fig);


    fprintf("  Candidate %d complete.\n",candidate);

end


%% ========================================================================
% 11. CREATE SUMMARY TABLE
% =========================================================================

summaryTable = table( ...
    candidateID, ...
    L2_mm, ...
    L3_mm, ...
    totalMainLink_mm, ...
    linkRatio, ...
    kinematicMaxReach_mm, ...
    collisionFreeMaxReach_mm, ...
    environmentMaxReach_mm, ...
    minHorizontalRadius_mm, ...
    xSpan_mm, ...
    ySpan_mm, ...
    zSpan_mm, ...
    collisionFreePercent, ...
    environmentValidPercent, ...
    selfCollisionPercent, ...
    worldCollisionPercent);


summaryTable.Properties.VariableNames = { ...
    'Candidate', ...
    'L2_mm', ...
    'L3_mm', ...
    'L2_plus_L3_mm', ...
    'L2_L3_Ratio', ...
    'Kinematic_Max_Reach_mm', ...
    'SelfCollisionFree_Max_Reach_mm', ...
    'Environment_Max_Reach_mm', ...
    'Minimum_Horizontal_Radius_mm', ...
    'Environment_X_Span_mm', ...
    'Environment_Y_Span_mm', ...
    'Environment_Z_Span_mm', ...
    'SelfCollisionFree_Percent', ...
    'EnvironmentValid_Percent', ...
    'SelfCollision_Percent', ...
    'WorldCollision_Percent' ...
};


%% ========================================================================
% 12. DISPLAY TABLE
% =========================================================================

fprintf("\n");
fprintf("============================================================\n");
fprintf("CANDIDATE COMPARISON\n");
fprintf("============================================================\n\n");

disp(summaryTable);


%% ========================================================================
% 13. EXPORT EXCEL
% =========================================================================

excelFile = fullfile( ...
    outputFolder, ...
    "Mechanical_Link_Length_Comparison.xlsx");


writetable( ...
    summaryTable, ...
    excelFile);


%% ========================================================================
% 14. EXPORT CSV
% =========================================================================

csvFile = fullfile( ...
    outputFolder, ...
    "Mechanical_Link_Length_Comparison.csv");


writetable( ...
    summaryTable, ...
    csvFile);


%% ========================================================================
% 15. SAVE COMPLETE MATLAB DATA
% =========================================================================

matFile = fullfile( ...
    outputFolder, ...
    "Mechanical_Handoff_Workspace_Results.mat");


save( ...
    matFile, ...
    "summaryTable", ...
    "allResults", ...
    "station", ...
    "candidates", ...
    "numSamples", ...
    "randomSeed");


%% ========================================================================
% 16. MAXIMUM REACH COMPARISON PLOT
% =========================================================================

fig = figure( ...
    "Visible","off");


bar( ...
    candidateID, ...
    environmentMaxReach_mm);


xlabel("Candidate");

ylabel("Environment-Constrained Maximum Reach [mm]");

title("Maximum Reach Comparison");

grid on


exportgraphics( ...
    fig, ...
    fullfile( ...
        plotFolder, ...
        "All_Candidates_Maximum_Reach_Comparison.png"), ...
    "Resolution",300);


close(fig);


%% ========================================================================
% 17. COLLISION-FREE PERCENTAGE COMPARISON
% =========================================================================

fig = figure( ...
    "Visible","off");


bar( ...
    candidateID, ...
    collisionFreePercent);


xlabel("Candidate");

ylabel("Collision-Free Random Configurations [%]");

title("Self-Collision Diagnostic Comparison");

grid on


exportgraphics( ...
    fig, ...
    fullfile( ...
        plotFolder, ...
        "All_Candidates_Self_Collision_Comparison.png"), ...
    "Resolution",300);


close(fig);


%% ========================================================================
% 18. L2/L3 DESIGN MAP
% =========================================================================

fig = figure( ...
    "Visible","off");


scatter( ...
    L2_mm, ...
    L3_mm, ...
    100, ...
    environmentMaxReach_mm, ...
    "filled");


xlabel("L2 [mm]");

ylabel("L3 [mm]");

title("L2 / L3 Candidate Design Map");

cb = colorbar;

cb.Label.String = ...
    "Environment-Constrained Maximum Reach [mm]";


grid on


for i = 1:numCandidates

    text( ...
        L2_mm(i)+5, ...
        L3_mm(i), ...
        sprintf("C%d",i));

end


exportgraphics( ...
    fig, ...
    fullfile( ...
        plotFolder, ...
        "L2_L3_Candidate_Design_Map.png"), ...
    "Resolution",300);


close(fig);


%% ========================================================================
% 19. CREATE HANDOFF NOTES
% =========================================================================

notesFile = fullfile( ...
    outputFolder, ...
    "README_MECHANICAL_HANDOFF.txt");


fid = fopen(notesFile,"w");


fprintf(fid, ...
    "GRADUATION PROJECT - SIMULATION TO MECHANICAL HANDOFF\n");

fprintf(fid, ...
    "=====================================================\n\n");


fprintf(fid, ...
    "Purpose:\n");

fprintf(fid, ...
    "Preliminary comparison of welding-robot L2/L3 geometry candidates.\n\n");


fprintf(fid, ...
    "IMPORTANT LIMITATIONS:\n");

fprintf(fid, ...
    "- L2 and L3 are current primary design variables.\n");

fprintf(fid, ...
    "- Base and wrist dimensions currently use UR5-scale preliminary values.\n");

fprintf(fid, ...
    "- Collision geometry is simplified and not CAD-derived.\n");

fprintf(fid, ...
    "- Welding table geometry is preliminary.\n");

fprintf(fid, ...
    "- TIG torch geometry is preliminary.\n");

fprintf(fid, ...
    "- Environment-valid percentage is NOT a robot performance score.\n");

fprintf(fid, ...
    "- Final link selection has NOT yet been frozen.\n");

fprintf(fid, ...
    "- Final selection will also consider Control-team IK, task reachability,\n");

fprintf(fid, ...
    "  welding orientation, singularity/manipulability and detailed CAD.\n\n");


fprintf(fid, ...
    "Files:\n");

fprintf(fid, ...
    "Mechanical_Link_Length_Comparison.xlsx\n");

fprintf(fid, ...
    "Mechanical_Link_Length_Comparison.csv\n");

fprintf(fid, ...
    "Mechanical_Handoff_Workspace_Results.mat\n");

fprintf(fid, ...
    "plots/ - workspace and comparison figures\n");

fprintf(fid, ...
    "robotPoses/ - robot configurations at maximum valid reach\n");


fclose(fid);


%% ========================================================================
% 20. FINISHED
% =========================================================================

fprintf("\n============================================================\n");
fprintf("EXPORT COMPLETE\n");
fprintf("============================================================\n");

fprintf("\nMechanical handoff package created at:\n");

fprintf("%s\n\n",outputFolder);

fprintf("Files created:\n");
fprintf("  Mechanical_Link_Length_Comparison.xlsx\n");
fprintf("  Mechanical_Link_Length_Comparison.csv\n");
fprintf("  Mechanical_Handoff_Workspace_Results.mat\n");
fprintf("  README_MECHANICAL_HANDOFF.txt\n");
fprintf("  plots/*.png\n");
fprintf("  robotPoses/*.png\n");

fprintf("\n============================================================\n");