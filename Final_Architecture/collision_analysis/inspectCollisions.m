function session = inspectCollisions(robot,overrides)
% INSPECTCOLLISIONS Interactively inspect sampled collision configurations.
%
% Command Window:
%   inspectCollisions
%
% The selected robot comes from projectConfig.m. The viewer shows collision
% geometry, not the cosmetic visual geometry.

% Inspector settings. These affect only this interactive inspection run.
settings.numSamples = 500;
settings.seed = 7;
settings.maxDisplayedCases = 60;
settings.minimumClearance = NaN; % NaN uses the robot-specific recommendation
settings.visible = true;

if nargin < 1 || isempty(robot)
    rootDir = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(rootDir));
    config = projectConfig();
    robot = loadRobot(config.robotID);
end
if nargin < 2
    overrides = struct();
end
settings = applyOverrides(settings,overrides);
if isnan(settings.minimumClearance)
    settings.minimumClearance = recommendedClearance(robot);
end

q = sampleConfigurations(robot,settings.numSamples,settings.seed);
options.minimumClearance = settings.minimumClearance;
options.showProgress = settings.numSamples >= 1000;
report = analyzeCollisions(robot,q,options);
indices = inspectionIndices(report,settings.maxDisplayedCases);
labels = caseLabels(report,indices);

visibility = matlab.lang.OnOffSwitchState(settings.visible);
fig = figure( ...
    'Name',char("Collision Inspector - "+string(robot.id)), ...
    'Color','w','Visible',visibility, ...
    'NumberTitle','off','Position',[120 80 1050 720]);
ax = axes(fig,'Position',[0.06 0.16 0.68 0.78]);
statusText = uicontrol(fig,'Style','text', ...
    'Units','normalized','Position',[0.06 0.03 0.68 0.09], ...
    'BackgroundColor','w','HorizontalAlignment','left', ...
    'FontSize',10);
popup = uicontrol(fig,'Style','popupmenu','String',cellstr(labels), ...
    'Units','normalized','Position',[0.77 0.78 0.20 0.06], ...
    'Callback',@selectCase);
uicontrol(fig,'Style','pushbutton','String','Previous', ...
    'Units','normalized','Position',[0.77 0.69 0.095 0.06], ...
    'Callback',@previousCase);
uicontrol(fig,'Style','pushbutton','String','Next', ...
    'Units','normalized','Position',[0.875 0.69 0.095 0.06], ...
    'Callback',@nextCase);
uicontrol(fig,'Style','text','String',sprintf( ...
    '%d tested\n%d colliding\n%d collision-free\n%d below %.1f mm clearance', ...
    report.summary.sampleCount,report.summary.collisionCount, ...
    report.summary.collisionFreeCount,report.summary.nearCollisionCount, ...
    1000*settings.minimumClearance), ...
    'Units','normalized','Position',[0.77 0.45 0.20 0.18], ...
    'BackgroundColor','w','HorizontalAlignment','left','FontSize',10);

currentCase = 1;
drawCase();

session.robot = robot;
session.settings = settings;
session.report = report;
session.displayedSampleIndices = indices;
session.figure = fig;

    function selectCase(source,~)
        currentCase = source.Value;
        drawCase();
    end

    function previousCase(~,~)
        currentCase = max(1,currentCase-1);
        popup.Value = currentCase;
        drawCase();
    end

    function nextCase(~,~)
        currentCase = min(numel(indices),currentCase+1);
        popup.Value = currentCase;
        drawCase();
    end

    function drawCase()
        sampleIndex = indices(currentCase);
        cla(ax);
        show(robot.model,report.q(sampleIndex,:), ...
            'Parent',ax,'Visuals','off','Collisions','on', ...
            'Frames','off','PreservePlot',false);
        hold(ax,'on');
        witness = report.witnessPoints(sampleIndex,:);
        if all(isfinite(witness))
            plot3(ax,witness([1 4]),witness([2 5]),witness([3 6]), ...
                'r-','LineWidth',2,'Marker','o','MarkerSize',5);
        end
        axis(ax,'equal');
        grid(ax,'on');
        view(ax,135,25);
        xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');

        pair = report.closestBodyPair(sampleIndex,:);
        if report.isSelfColliding(sampleIndex)
            state = "COLLISION";
            clearanceText = "intersecting";
            color = [0.65 0.05 0.05];
        elseif report.isNearCollision(sampleIndex)
            state = "NEAR COLLISION";
            clearanceText = sprintf('%.2f mm', ...
                1000*report.minimumClearance(sampleIndex));
            color = [0.85 0.40 0.00];
        else
            state = "CLEAR";
            clearanceText = sprintf('%.2f mm', ...
                1000*report.minimumClearance(sampleIndex));
            color = [0.00 0.45 0.20];
        end
        title(ax,sprintf('%s | sample %d of %d',state,sampleIndex, ...
            report.summary.sampleCount),'Color',color);
        statusText.String = sprintf([ ...
            'State: %s\nClosest/reported pair: %s and %s\nClearance: %s\n', ...
            'q (deg): %s'],state,pair(1),pair(2),clearanceText, ...
            mat2str(rad2deg(report.q(sampleIndex,:)),4));
    end

end

function settings = applyOverrides(settings,overrides)

names = fieldnames(overrides);
for i = 1:numel(names)
    if ~isfield(settings,names{i})
        error('collision:UnknownInspectorSetting', ...
            'Unknown inspector setting "%s".',names{i});
    end
    settings.(names{i}) = overrides.(names{i});
end

end

function clearance = recommendedClearance(robot)

clearance = 0.010;
if isfield(robot,'collision') && isstruct(robot.collision) && ...
        isfield(robot.collision,'minimumClearance')
    clearance = robot.collision.minimumClearance;
end

end

function q = sampleConfigurations(robot,numSamples,seed)

limits = robot.params.joints.positionLimits;
previousState = rng;
cleanup = onCleanup(@() rng(previousState));
rng(seed,'twister');
randomSamples = limits(:,1).'+rand(max(0,numSamples-1), ...
    robot.structure.dof).*(limits(:,2)-limits(:,1)).';
q = [robot.params.joints.homePosition;randomSamples];

end

function indices = inspectionIndices(report,maxCount)

colliding = find(report.isSelfColliding);
near = find(report.isNearCollision);
clear = find(report.hasRequiredClearance);
[~,nearOrder] = sort(report.minimumClearance(near),'ascend');
[~,clearOrder] = sort(report.minimumClearance(clear),'ascend');
indices = unique([1;colliding;near(nearOrder);clear(clearOrder)],'stable');
indices = indices(1:min(maxCount,numel(indices)));

end

function labels = caseLabels(report,indices)

labels = strings(numel(indices),1);
for i = 1:numel(indices)
    sampleIndex = indices(i);
    if report.isSelfColliding(sampleIndex)
        state = "COLLISION";
    elseif report.isNearCollision(sampleIndex)
        state = "NEAR";
    else
        state = "CLEAR";
    end
    labels(i) = sprintf('%s - sample %d',state,sampleIndex);
end

end
