function session = park_position_analysis()
% PARK_POSITION_ANALYSIS Compare practical, compact, low-holding-load poses.
% No arguments. Robot selection remains in projectConfig.m.
settings.gravity = [0 0 -9.81];
settings.numSamples = 1000;
settings.numStarts = 4;
settings.seed = 23;
settings.minimumClearance = .010; % Self-clearance, metres.
settings.allowableHoldingTorque_Nm = []; % Joint-output continuous ratings, J1..Jn.
% Empty ratings: minimize peak N m; no motor-utilization percentages invented.
settings.maximumHeight_m = []; % Empty uses this robot's park geometry settings.
settings.maximumRadius_m = [];
settings.jointStopMargin_deg = 5;
settings.localSpread_deg = []; % Empty uses robot-specific local sampling ranges.
settings.maxFunctionEvaluations = 350;
settings.maxIterations = 60;
settings.mountingPlaneZ_m = 0; % Base mounting plane, in robot-base axes.
settings.maxSeedCollisionChecks = 60;
settings.maxCollisionRefinements = 2;
settings.patternRefinements = 1; % Optional Global Optimization Toolbox polish.
settings.numCandidates = 4;
settings.checkPaths = true;
settings.pathStep_deg = 2; % Maximum per-joint increment for sampled checks.
settings.useRRT = false; % Optional slower MATLAB planner; at most one attempt.
settings.maxRRTAttempts = 1;
settings.rrtIterations = 100;
settings.showProgress = true;

root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
robot = loadRobot();
previewModel=copy(robot.model); previewModel.Gravity=settings.gravity;
[rootMass,rootCOM]=rootMassProperties(robot);
bodyMass=sum(cellfun(@(body) body.Mass,robot.model.Bodies));
totalMass=bodyMass+rootMass;
uprightComparison=[];
q = reshape(robot.params.joints.homePosition,1,[]);
result = [];
displayed = [];
selected = 0;
plane = gobjects(0);
fig = figure('Name',sprintf('%s | Park position analysis',robot.id), ...
    'NumberTitle','off','Color','w','Position',[40 40 1550 950]);
setappdata(fig,'ParkSearchBusy',false);
label(fig,[.03 .945 .94 .04],sprintf('%s | Park position candidates',robot.id),16);
robotAx = axes(fig,'Position',[.04 .35 .45 .51]);
show(robot.model,q,'Parent',robotAx,'FastUpdate',true,'Frames','off','PreservePlot',false);
axis(robotAx,'equal'); grid(robotAx,'on'); view(robotAx,135,25);
xlabel(robotAx,'X (m)'); ylabel(robotAx,'Y (m)'); zlabel(robotAx,'Z (m)');
set(robotAx,'PositionConstraint','outerposition','OuterPosition',[.02 .38 .49 .52]);
cogAx=axes(fig,'Position',robotAx.Position,'Color','none', ...
    'HandleVisibility','off','HitTest','off','PickableParts','none');
axis(cogAx,'off'); hold(cogAx,'on');
cogDot=plot3(cogAx,NaN,NaN,NaN,'o','MarkerSize',10, ...
    'MarkerFaceColor',[.9 .12 .25],'MarkerEdgeColor','k', ...
    'HitTest','off','PickableParts','none','Tag','ParkWholeRobotCOG');
cogLabel=text(cogAx,NaN,NaN,NaN,'CoG (incl. base)', ...
    'FontSize',10,'BackgroundColor','w','Margin',2,'VerticalAlignment','bottom', ...
    'HitTest','off','PickableParts','none','Clipping','off');
cameraProperties={'CameraPosition','CameraTarget','CameraUpVector', ...
    'CameraViewAngle','Projection','XLim','YLim','ZLim', ...
    'DataAspectRatio','PlotBoxAspectRatio'};
for property=cameraProperties
    set(cogAx,property{1},get(robotAx,property{1}));
end
cameraLink=linkprop([robotAx cogAx],cameraProperties);
positionListener=addlistener(robotAx,'Position','PostSet', ...
    @(~,~) set(cogAx,'Position',robotAx.Position));
setappdata(fig,'ParkCOGLinks',{cameraLink,positionListener});
updateCOG();
view(robotAx,135,25);
rotate3d(fig,'on');
candidateSelect = uicontrol(fig,'Style','popupmenu','Units','normalized', ...
    'Position',[.54 .885 .43 .04],'String',{'Searching...'},'Tag','ParkCandidate', ...
    'Callback',@(source,~) selectCandidate(source.Value));
displayLabel = label(fig,[.54 .825 .43 .055],'Searching for park poses...',13);
displayLabel.Tag = 'DisplayedParkTorque';
torqueAx = axes(fig,'Position',[.595 .60 .36 .18],'Tag','ParkTorqueAxes');
compare = uicontrol(fig,'Style','checkbox','Units','normalized', ...
    'Position',[.54 .535 .43 .035],'BackgroundColor','w','Value',0, ...
    'String','Compare with configured home (gray)','Tag','ParkCompareHome', ...
    'Callback',@(~,~) drawValues());
tableView = uitable(fig,'Units','normalized','Position',[.54 .30 .43 .215], ...
    'ColumnName',{'Joint','Displayed angle (deg)','Displayed torque (N m)'}, ...
    'ColumnWidth',{110 185 185},'Data',cell(0,3),'Tag','ParkJointValues');
summary = label(fig,[.54 .16 .43 .115],'',11);
stability = label(fig,[.54 .085 .43 .07],'',11);
shortlist = uitable(fig,'Units','normalized','Position',[.03 .10 .47 .17], ...
    'ColumnName',{'Candidate','J2 (N m)','J3 (N m)','Peak (N m)'}, ...
    'ColumnWidth',{260 105 105 105},'Data',cell(0,4), ...
    'CellSelectionCallback',@selectRow,'Tag','ParkShortlist');
homeButton = button(fig,[.03 .285 .16 .04],'Show configured home',@showReference);
parkButton = button(fig,[.20 .285 .16 .04],'Show preferred park',@showPark);
pathButton = button(fig,[.37 .285 .13 .04],'Preview return',@previewReturn);
searchButton = button(fig,[.54 .035 .17 .04],'Search again',@runSearch);
status = label(fig,[.03 .015 .47 .065],'Starting search...',10);
controls = [candidateSelect compare homeButton parkButton pathButton searchButton];
session.figure = fig;
session.robot = robot;
session.getState = @getState;
session.showPark = @showPark;
session.showReference = @showReference;
session.showUprightComparison = @showUprightComparison;
session.selectCandidate = @selectCandidate;
session.search = @runSearch;
session.previewReturn = @previewReturn;
session.close = @() close(fig);
runSearch();

    function runSearch()
        if getappdata(fig,'ParkSearchBusy'), return; end
        setappdata(fig,'ParkSearchBusy',true);
        set(controls,'Enable','off');
        cleanup = onCleanup(@() enableControls(fig,controls));
        status.String = 'Searching compact postures, holding loads, self-clearance and return routes...';
        drawnow;
        result = optimizeParkConfiguration(robot,settings);
        uprightComparison=[];
        if isfield(robot.params,'park') && isfield(robot.params.park,'uprightComparisonConfiguration')
            uprightComparison=comparisonPose(robot,previewModel,result.reference, ...
                robot.params.park.uprightComparisonConfiguration,settings);
        end
        if ~result.found
            selected = 0; displayed = [];
            candidateSelect.String = {'No qualifying pose'};
            tableView.Data = cell(0,3); shortlist.Data = cell(0,4);
            displayLabel.String = 'No qualifying pose found above the mounting plane.';
            summary.String = ''; stability.String = ''; cla(torqueAx);
            status.String = 'No qualifying pose found in this search. Configured home unchanged.';
            return;
        end
        choiceNames = [result.candidates.name];
        for k=1:numel(result.candidates)
            if result.settings.checkPaths && ~result.candidates(k).returnVerified
                choiceNames(k)=choiceNames(k)+" [pose only]";
            end
        end
        span = diff(robot.params.joints.positionLimits,1,2).';
        for k = 2:numel(result.candidates)
            previous = result.candidates(1:k-1);
            if any(arrayfun(@(c) norm((c.pose.q-result.candidates(k).pose.q)./span)<1e-6,previous))
                choiceNames(k) = choiceNames(k)+" (same pose)";
            end
        end
        menuNames = [cellstr(choiceNames) {'Configured home (reference)'}];
        if ~isempty(uprightComparison)
            menuNames=[menuNames ...
                {'Upright near-zero torque (unstable; comparison only)'}];
        end
        candidateSelect.String=menuNames;
        candidateSelect.Value = 1;
        rows = cell(numel(result.candidates),4);
        for k = 1:numel(result.candidates)
            c = result.candidates(k);
            t = c.pose.torque_Nm;
            rows(k,:) = {char(choiceNames(k)),jointValue(t,2),jointValue(t,3),sprintf('%.5g',max(abs(t)))};
        end
        shortlist.Data = rows;
        if ~isempty(uprightComparison)
            t=uprightComparison.torque_Nm;
            shortlist.Data(end+1,:)={'Upright unstable (comparison)', ...
                jointValue(t,2),jointValue(t,3),sprintf('%.5g',max(abs(t)))};
        end
        status.String = sprintf('%.1f s | whole visual model above Z=%.3f m | configured home unchanged.', ...
            result.statistics.elapsedSeconds,settings.mountingPlaneZ_m);
        if ~result.meta.motorRatingsAvailable
            status.String = ['Ranked in N m; motor ratings not supplied. ' status.String];
        end
        if result.meta.preliminary, status.String = ['PRELIMINARY | ' status.String]; end
        if result.settings.checkPaths && ~result.returnVerified
            status.String=['POSES ONLY: no return verified from every valid start. ' status.String];
        end
        showPark();
    end

    function selectCandidate(index)
        if isempty(result) || ~result.found, return; end
        if ischar(index) || isstring(index)
            if string(index)=="upright_comparison", showUprightComparison(); return; end
            index = find([result.candidates.id]==string(index),1);
        end
        validateattributes(index,{'numeric'},{'scalar','integer','>=',1,'<=', ...
            numel(result.candidates)+1+double(~isempty(uprightComparison))});
        if index==numel(result.candidates)+1, showReference(); return; end
        if index==numel(result.candidates)+2, showUprightComparison(); return; end
        selected = index;
        candidateSelect.Value = index;
        displayed = result.candidates(index).pose;
        render(char(result.candidates(index).name));
    end

    function selectRow(~,event)
        if getappdata(fig,'ParkSearchBusy') || isempty(event.Indices), return; end
        if event.Indices(1,1)>numel(result.candidates), showUprightComparison(); return; end
        selectCandidate(event.Indices(1,1));
    end

    function showPark()
        selectCandidate(1);
    end

    function showReference()
        if isempty(result), return; end
        selected = 0;
        candidateSelect.Value = numel(result.candidates)+1;
        displayed = result.reference;
        render('Configured home (reference)');
    end

    function showUprightComparison()
        if isempty(uprightComparison), return; end
        selected=-1;
        candidateSelect.Value=numel(result.candidates)+2;
        displayed=uprightComparison;
        render('Upright near-zero torque - unstable comparison');
    end

    function previewReturn()
        if isempty(result) || selected<=0, return; end
        routes=result.candidates(selected).paths;
        if isempty(routes) || ~any([routes.passed])
            status.String='No checked return route for this candidate. Pose preview only.';
            return;
        end
        route=routes(find([routes.passed],1));
        set(controls,'Enable','off');
        cleanup=onCleanup(@() enableControls(fig,controls));
        status.String=sprintf('Return preview: %s | sampled at <= %.1f deg per joint. Not a hardware command.', ...
            route.method,route.maxStep_deg);
        savedPose=displayed;
        summary.String='Return preview: instantaneous static holding loads, not dynamic motion torques.';
        for k=1:size(route.q,1)
            if ~isgraphics(fig), return; end
            q=route.q(k,:); displayed.q=q;
            displayed.torque_Nm=gravityTorque(previewModel,q);
            show(robot.model,q,'Parent',robotAx,'FastUpdate',true, ...
                'Frames','off','PreservePlot',false);
            updateCOG();
            displayLabel.String=sprintf('Return preview: J2 %s N m | J3 %s N m (static)', ...
                jointValue(displayed.torque_Nm,2),jointValue(displayed.torque_Nm,3));
            drawValues();
            drawnow; pause(.025);
        end
        displayed=savedPose;
        render(char(result.candidates(selected).name));
    end

    function render(name)
        q = displayed.q;
        show(robot.model,q,'Parent',robotAx,'FastUpdate',true,'Frames','off','PreservePlot',false);
        bounds = robotPoseBounds(robot,[result.reference.q;q]);
        bounds(5) = min(bounds(5),settings.mountingPlaneZ_m-.05);
        axis(robotAx,bounds); title(robotAx,name,'FontSize',12);
        set(robotAx,'PositionConstraint','outerposition','OuterPosition',[.02 .38 .49 .52]);
        if isgraphics(plane), delete(plane); end
        hold(robotAx,'on');
        plane = patch(robotAx,[bounds(1) bounds(2) bounds(2) bounds(1)], ...
            [bounds(3) bounds(3) bounds(4) bounds(4)],settings.mountingPlaneZ_m*ones(1,4), ...
            [.65 .7 .72],'FaceAlpha',.12,'EdgeColor',[.5 .55 .55], ...
            'HitTest','off','PickableParts','none');
        hold(robotAx,'off');
        updateCOG();
        displayLabel.String = sprintf('Displayed pose: J2 %s N m | J3 %s N m', ...
            jointValue(displayed.torque_Nm,2),jointValue(displayed.torque_Nm,3));
        drawValues();
        routeSummary='Reference pose; not a return-route recommendation.';
        if selected<0, routeSummary='Historical comparison only; outside park envelope; return not checked.'; end
        if selected>0
            routes=result.candidates(selected).paths;
            routeSummary=sprintf('Sampled return routes: %d / %d valid starts passed.', ...
                nnz([routes.passed]),nnz([result.workingStarts.valid]));
            if ~result.settings.checkPaths, routeSummary='Return routes not checked.'; end
        end
        ratingSummary='Motor utilization: unavailable until joint-output ratings are supplied.';
        if result.meta.motorRatingsAvailable
            ratingSummary=sprintf('Largest continuous holding utilization: %.2f%%',100*max(displayed.utilization));
        end
        summary.String = sprintf(['Torque norm: %.4g N m | peak joint: %.4g N m\n' ...
            'Envelope [X Y Z]: %s m\n' ...
            'Self-clearance: %.1f mm | minimum Z: %.4f m\n%s\n%s'], ...
            displayed.torqueNorm_Nm,displayed.peakAbsTorque_Nm, ...
            mat2str(displayed.plane.extent_m,3),1000*displayed.collision.minimumClearance, ...
            displayed.plane.minimumZ_m,routeSummary,ratingSummary);
        stability.String = char(displayed.stability.label);
        stability.ForegroundColor = [.15 .35 .22];
        if ~displayed.stability.passesGravityCheck
            stability.ForegroundColor = [.75 .20 .05];
        end
        if ~displayed.plane.abovePlane || ~displayed.collision.hasRequiredClearance
            stability.String = ['REFERENCE FAILS GEOMETRY CONSTRAINTS. ' stability.String];
            stability.ForegroundColor = [.75 .05 .05];
        elseif selected>0 && result.settings.checkPaths && ~result.candidates(selected).returnVerified
            stability.String = ['RETURN NOT VERIFIED FROM ALL STARTS. ' stability.String];
            stability.ForegroundColor = [.75 .05 .05];
        end
        if selected<0
            stability.String='GRAVITY-UNSTABLE UPRIGHT COMPARISON. Active holding/brakes required; not a recommended park.';
            stability.ForegroundColor=[.75 .05 .05];
        end
        drawnow;
    end

    function updateCOG()
        cog=(bodyMass*centerOfMass(robot.model,q)+rootMass*rootCOM)/totalMass;
        cogDot.XData=cog(1); cogDot.YData=cog(2); cogDot.ZData=cog(3);
        cogDot.UserData=struct('position_m',cog,'totalMass_kg',totalMass, ...
            'rootMass_kg',rootMass,'includesFixedBase',true);
        cogLabel.Position=cog+[0 0 .035];
        cogAx.Position=robotAx.Position;
    end

    function drawValues()
        if isempty(displayed), return; end
        names = cellstr(string(robot.structure.jointNames(:)));
        rows = [names cell(robot.structure.dof,2)];
        for j = 1:robot.structure.dof
            rows(j,2:3) = {sprintf('%+.3f',rad2deg(displayed.q(j))), ...
                sprintf('%+.6f',displayed.torque_Nm(j))};
        end
        tableView.Data = rows;
        cla(torqueAx);
        legend(torqueAx,'off');
        if compare.Value
            b = bar(torqueAx,[abs(displayed.torque_Nm(:)) abs(result.reference.torque_Nm(:))]);
            b(1).FaceColor = [.15 .48 .34]; b(2).FaceColor = [.65 .67 .69];
            legend(torqueAx,{'Displayed pose','Configured home'},'Location','best');
        else
            bar(torqueAx,abs(displayed.torque_Nm),'FaceColor',[.15 .48 .34]);
        end
        torqueAx.XTick = 1:robot.structure.dof; torqueAx.XTickLabel = names;
        torqueAx.TickLabelInterpreter = 'none';
        torqueAx.Tag = 'ParkTorqueAxes';
        ylabel(torqueAx,'Absolute holding torque (N m)');
        title(torqueAx,'Displayed configuration'); grid(torqueAx,'on');
    end

    function state = getState()
        state = struct('q',q,'result',result,'settings',settings, ...
            'selectedCandidate',selected,'displayedPose',displayed, ...
            'uprightComparison',uprightComparison,'centerOfGravity',cogDot.UserData);
    end
end

function value = jointValue(t,j)
if numel(t)>=j, value = sprintf('%+.6f',t(j)); else, value = 'n/a'; end
end

function h = label(parent,position,value,size)
h = uicontrol(parent,'Style','text','Units','normalized','Position',position, ...
    'String',value,'FontSize',size,'BackgroundColor','w','HorizontalAlignment','left');
end

function h = button(parent,position,value,action)
h = uicontrol(parent,'Style','pushbutton','Units','normalized','Position',position, ...
    'String',value,'Callback',@(~,~) action());
end

function enableControls(fig,controls)
if ~isgraphics(fig), return; end
setappdata(fig,'ParkSearchBusy',false);
set(controls(isgraphics(controls)),'Enable','on');
end

function [mass,com]=rootMassProperties(robot)
% Imported URDF root inertials are not accessible on rigidBodyTree.Base.
% Programmatic models keep their fixed base as an ordinary body instead.
mass=0; com=[0 0 0];
if ~isfield(robot.params,'source') || ~isfield(robot.params.source,'urdf'), return; end
document=xmlread(char(robot.params.source.urdf));
links=document.getElementsByTagName('link');
for k=0:links.getLength-1
    node=links.item(k);
    if ~strcmp(char(node.getAttribute('name')),robot.model.BaseName), continue; end
    inertials=node.getElementsByTagName('inertial');
    if inertials.getLength==0, return; end
    inertial=inertials.item(0);
    mass=str2double(char(inertial.getElementsByTagName('mass').item(0).getAttribute('value')));
    origins=inertial.getElementsByTagName('origin');
    if origins.getLength>0 && origins.item(0).hasAttribute('xyz')
        com=sscanf(char(origins.item(0).getAttribute('xyz')),'%f').';
    end
    validateattributes(mass,{'numeric'},{'scalar','finite','nonnegative'});
    validateattributes(com,{'numeric'},{'row','numel',3,'finite'});
    return;
end
error('park:MissingRootLink','Cannot find the URDF root link for whole-robot CoG.');
end

function pose=comparisonPose(robot,model,reference,q,settings)
q=reshape(q,1,[]);
validateattributes(q,{'numeric'},{'row','numel',robot.structure.dof,'finite'});
limits=robot.params.joints.positionLimits;
assert(all(q>=limits(:,1).' & q<=limits(:,2).'),'park:ComparisonLimits', ...
    'The historical upright comparison is outside the current joint limits.');
pose=reference; pose.q=q; pose.source="historical upright comparison only";
pose.torque_Nm=gravityTorque(model,q); pose.torqueNorm_Nm=norm(pose.torque_Nm);
pose.peakAbsTorque_Nm=max(abs(pose.torque_Nm)); pose.objective=pose.peakAbsTorque_Nm;
pose.utilization=NaN(size(q));
if ~isempty(settings.allowableHoldingTorque_Nm)
    pose.utilization=abs(pose.torque_Nm)./reshape(settings.allowableHoldingTorque_Nm,1,[]);
    pose.objective=max(pose.utilization);
end
pose.collision=analyzeCollisions(robot,q,struct('minimumClearance',settings.minimumClearance));
bodies=[{model.Base} model.Bodies]; minimum=Inf(1,3); maximum=-Inf(1,3);
heights=Inf(numel(bodies),1); radius=0; energy=0;
for b=1:numel(bodies)
    body=bodies{b}; T=eye(4);
    if b>1
        T=getTransform(model,q,body.Name);
        center=T(1:3,1:3)*body.CenterOfMass.'+T(1:3,4);
        energy=energy-body.Mass*dot(model.Gravity,center);
    end
    visuals=getVisual(body);
    for v=1:numel(visuals)
        mesh=visuals(v); P=mesh.Triangulation.Points.*mesh.Scale;
        Tvisual=T*mesh.Tform; P=P*Tvisual(1:3,1:3).'+Tvisual(1:3,4).';
        minimum=min(minimum,min(P,[],1)); maximum=max(maximum,max(P,[],1));
        heights(b)=min(heights(b),min(P(:,3)));
        radius=max(radius,max(vecnorm(P(:,1:2),2,2)));
    end
end
pose.energy_J=energy; pose.plane.energy_J=energy;
pose.plane.com_m=centerOfMass(model,q); pose.plane.torque_Nm=pose.torque_Nm;
pose.plane.minimumZ_m=minimum(3); pose.plane.maximumZ_m=maximum(3);
pose.plane.bodyMinimumZ_m=heights;
pose.plane.abovePlane=minimum(3)>=settings.mountingPlaneZ_m-1e-6;
pose.plane.extent_m=maximum-minimum; pose.plane.diagonal_m=norm(maximum-minimum);
pose.plane.radius_m=radius;
H=zeros(numel(q),numel(q));
for j=1:numel(q)
    a=q; b=q; a(j)=max(limits(j,1),q(j)-1e-4); b(j)=min(limits(j,2),q(j)+1e-4);
    H(:,j)=(gravityTorque(model,b)-gravityTorque(model,a)).'/(b(j)-a(j));
end
H=(H+H.')/2; eigenvalues=eig(H);
pose.stability=struct('potentialHessian',H,'eigenvalues',eigenvalues, ...
    'passiveEquilibrium',max(abs(pose.torque_Nm))<1e-3 && min(eigenvalues)>=-1e-4, ...
    'passesGravityCheck',min(eigenvalues)>=-1e-4, ...
    'label',"Historical upright comparison; active holding/brakes required.");
end
