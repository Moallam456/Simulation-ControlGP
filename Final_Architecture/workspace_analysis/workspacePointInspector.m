function inspector = workspacePointInspector( ...
        robot,fig,workspaceAxes,plotItems,data,setRobotPose)
% WORKSPACEPOINTINSPECTOR Inspect the configurations behind plotted points.

panel = uipanel(fig,'Title','Point inspector','Units','normalized', ...
    'Position',[0.65 0.05 0.33 0.50],'BackgroundColor','w');
clickedText = uicontrol(panel,'Style','text','Units','normalized', ...
    'Position',[0.04 0.87 0.92 0.11],'BackgroundColor','w', ...
    'HorizontalAlignment','left','String','Click a workspace point.');
positionText = uicontrol(panel,'Style','text','Units','normalized', ...
    'Position',[0.04 0.81 0.92 0.05],'BackgroundColor','w', ...
    'HorizontalAlignment','left','String','');
candidateList = uicontrol(panel,'Style','listbox','Units','normalized', ...
    'Position',[0.04 0.51 0.92 0.29],'String',{'No point selected'}, ...
    'Callback',@candidateChanged);
angleTable = uitable(panel,'Units','normalized', ...
    'Position',[0.04 0.23 0.92 0.26], ...
    'ColumnName',{'Joint','Angle (deg)'}, ...
    'ColumnEditable',[false false], ...
    'ColumnWidth',{155 100}, ...
    'Data',[cellstr(string(robot.structure.jointNames(:))) ...
        repmat({''},robot.structure.dof,1)]);
statusText = uicontrol(panel,'Style','text','Units','normalized', ...
    'Position',[0.04 0.12 0.92 0.10],'BackgroundColor','w', ...
    'HorizontalAlignment','left','String','Self-collision only; no path check.');
reachButton = uicontrol(panel,'Style','pushbutton','Units','normalized', ...
    'Position',[0.04 0.025 0.41 0.08],'String','Reach', ...
    'Enable','off','Callback',@(~,~) reach());
findButton = uicontrol(panel,'Style','pushbutton','Units','normalized', ...
    'Position',[0.52 0.025 0.44 0.08],'String','Find alternatives', ...
    'Enable','off','Callback',@(~,~) findAlternatives());

candidates = emptyCandidates(robot.structure.dof);
selectedIndex = 0;
clickedSampleIndex = 0;
selectedVoxel = zeros(1,3);
message = "";
setappdata(fig,'WorkspacePoseChanged',@updateDetails);
cursor = datacursormode(fig);
set(cursor,'Enable','on','UpdateFcn',@clickedMarker);

inspector.panel = panel;
inspector.list = candidateList;
inspector.reachButton = reachButton;
inspector.findButton = findButton;
inspector.selectPlotItem = @selectPlotItem;
inspector.selectSample = @selectSample;
inspector.selectVoxel = @selectVoxel;
inspector.selectCandidate = @selectCandidate;
inspector.reach = @reach;
inspector.findAlternatives = @findAlternatives;
inspector.getState = @getState;

    function tip = clickedMarker(~,event)
        tip = {'Workspace point'};
        for item = 1:numel(plotItems)
            if isequal(event.Target,plotItems(item).handle)
                selectPlotItem(item,event.DataIndex);
                if selectedIndex > 0
                    choice = candidates(selectedIndex);
                    tip = {sprintf('TCP [%.3f %.3f %.3f] m',choice.point), ...
                        sprintf('Selected ICN %.3f',choice.icn)};
                end
                return;
            end
        end
    end

    function selectPlotItem(itemIndex,pointIndex)
        if itemIndex < 1 || itemIndex > numel(plotItems) || ...
                pointIndex < 1 || pointIndex ~= floor(pointIndex)
            error('workspace:InvalidPointSelection','Invalid plotted point.');
        end
        item = plotItems(itemIndex);
        if item.kind == "sample"
            if pointIndex > numel(item.indices)
                error('workspace:InvalidPointSelection','Invalid sample marker index.');
            end
            selectSample(item.indices(pointIndex));
        else
            if pointIndex > size(item.voxelCoordinates,1)
                error('workspace:InvalidPointSelection','Invalid voxel marker index.');
            end
            selectVoxel(item.voxelCoordinates(pointIndex,:));
        end
    end

    function selectSample(sampleIndex)
        if sampleIndex < 1 || sampleIndex > size(data.q,1) || ...
                sampleIndex ~= floor(sampleIndex)
            error('workspace:InvalidSample','Sample index is out of range.');
        end
        clickedSampleIndex = sampleIndex;
        voxel = data.voxelCoordinates(sampleIndex,:);
        selectVoxelGroup(voxel);
    end

    function selectVoxel(voxel)
        if ~isequal(size(voxel),[1 3]) || any(~isfinite(voxel)) || ...
                any(voxel ~= floor(voxel))
            error('workspace:InvalidVoxel','Voxel must be a finite 1-by-3 index.');
        end
        clickedSampleIndex = 0;
        selectVoxelGroup(voxel);
    end

    function selectVoxelGroup(voxel)
        selectedVoxel = voxel;
        sampleIDs = find(all(data.voxelCoordinates == voxel,2));
        if isempty(sampleIDs)
            error('workspace:EmptyVoxel','No sampled configuration lies in this voxel.');
        end
        candidates = emptyCandidates(robot.structure.dof);
        icn = data.manipulability.values(sampleIDs);
        if all(isnan(icn))
            icn = zeros(numel(sampleIDs),1);
            for k = 1:numel(sampleIDs)
                icn(k) = linearICN(data.q(sampleIDs(k),:));
            end
        end
        if data.collision.status == "EVALUATED"
            free = data.collision.isCollisionFree(sampleIDs);
            clearance = data.collision.clearance_m(sampleIDs);
            pairs = data.collision.closestBodyPair(sampleIDs,:);
        else
            [free,clearance,pairs] = checkCandidateCollisions(data.q(sampleIDs,:));
        end
        for k = 1:numel(sampleIDs)
            id = sampleIDs(k);
            candidates(end+1) = makeCandidate( ...
                "sampled",id,data.q(id,:),data.transforms(:,:,id), ...
                icn(k),free(k),clearance(k),pairs(k,:),data.accepted(id)); %#ok<AGROW>
        end
        eligible = rankCandidates();
        if any(eligible)
            selectedIndex = 1;
        elseif clickedSampleIndex > 0
            selectedIndex = find([candidates.sampleIndex] == clickedSampleIndex,1);
        else
            [~,selectedIndex] = max([candidates.icn]);
        end
        if clickedSampleIndex > 0
            clickedQ = rad2deg(data.q(clickedSampleIndex,:));
            clickedText.String = {sprintf('Clicked sample #%d: [%.3f %.3f %.3f] m', ...
                clickedSampleIndex,data.points(clickedSampleIndex,:)), ...
                sprintf('Clicked angles (deg): %s',sprintf('%+.1f ',clickedQ))};
        else
            center = (voxel+0.5)*data.settings.voxelSize;
            clickedText.String = sprintf('Voxel center [%.3f %.3f %.3f] m; %d samples', ...
                center,numel(sampleIDs));
        end
        message = "";
        refreshList();
    end

    function eligible = rankCandidates()
        eligible = arrayfun(@(c) c.collisionFree == 1 && ...
            (data.analysisType ~= "usable" || ...
                c.icn >= data.settings.manipulabilityThreshold),candidates);
        quality = [candidates.icn].';
        clearanceRank = [candidates.clearance_m].';
        clearanceRank(~isfinite(clearanceRank)) = -1;
        [~,order] = sortrows([-double(eligible(:)) -quality ...
            -clearanceRank [candidates.sampleIndex].']);
        candidates = candidates(order);
        eligible = eligible(order);
    end

    function candidate = makeCandidate(source,id,q,T,icn,free,clearance,pair,accepted)
        candidate.source = source;
        candidate.sampleIndex = id;
        candidate.q = reshape(q,1,[]);
        candidate.transform = T;
        candidate.point = reshape(T(1:3,4),1,3);
        candidate.icn = icn;
        candidate.collisionFree = free;
        candidate.clearance_m = clearance;
        candidate.closestPair = reshape(string(pair),1,2);
        candidate.acceptedByPlot = accepted;
    end

    function [free,clearance,pairs] = checkCandidateCollisions(configurations)
        count = size(configurations,1);
        if ~isfield(robot,'collision') || ~isfield(robot.collision,'available') || ...
                ~robot.collision.available
            free = nan(count,1);
            clearance = nan(count,1);
            pairs = strings(count,2);
            return;
        end
        report = analyzeCollisions(robot,configurations, ...
            struct('showProgress',false));
        free = double(report.isCollisionFree);
        clearance = report.minimumClearance;
        pairs = report.closestBodyPair;
    end

    function value = linearICN(configuration)
        J = geometricJacobian(robot.model,configuration, ...
            char(robot.structure.frames.endEffector));
        sigma = svd(J(4:6,:),'econ');
        if isempty(sigma) || sigma(1) <= eps(max(1,sigma(1)))
            value = 0;
        else
            value = max(0,min(1,sigma(end)/sigma(1)));
        end
    end

    function refreshList()
        if isempty(candidates)
            candidateList.String = {'No candidates'};
            candidateList.Value = 1;
            reachButton.Enable = 'off';
            findButton.Enable = 'off';
            updateDetails();
            return;
        end
        rows = cell(1,numel(candidates));
        for k = 1:numel(candidates)
            c = candidates(k);
            if c.source == "sampled"
                label = sprintf('#%d sampled',c.sampleIndex);
            else
                label = sprintf('IK %d',k);
            end
            if isnan(c.collisionFree)
                collisionLabel = 'UNKNOWN';
            elseif c.collisionFree
                collisionLabel = 'FREE';
            else
                collisionLabel = 'COLLIDING';
            end
            if isfinite(c.clearance_m)
                clearanceLabel = sprintf('%.0f mm',1000*c.clearance_m);
            else
                clearanceLabel = 'n/a';
            end
            rows{k} = sprintf('%s | ICN %.3f | %s | %s | %s', ...
                label,c.icn,collisionLabel,clearanceLabel, ...
                filterLabel(c.acceptedByPlot));
        end
        candidateList.String = rows;
        candidateList.Value = selectedIndex;
        reachButton.Enable = 'on';
        findButton.Enable = 'on';
        updateDetails();
    end

    function label = filterLabel(accepted)
        if accepted
            label = 'passes filter';
        else
            label = 'filtered';
        end
    end

    function candidateChanged(~,~)
        selectCandidate(candidateList.Value);
    end

    function selectCandidate(index)
        if index < 1 || index > numel(candidates) || index ~= floor(index)
            error('workspace:InvalidCandidate','Candidate index is out of range.');
        end
        selectedIndex = index;
        candidateList.Value = index;
        message = "";
        updateDetails();
    end

    function updateDetails()
        if selectedIndex == 0 || isempty(candidates)
            positionText.String = '';
            statusText.String = 'Self-collision only; no path check.';
            return;
        end
        c = candidates(selectedIndex);
        positionText.String = sprintf('Selected TCP [%.3f %.3f %.3f] m',c.point);
        angleTable.Data = [cellstr(string(robot.structure.jointNames(:))) ...
            num2cell(rad2deg(c.q(:)))];
        pair = strjoin(c.closestPair(c.closestPair ~= ""),' / ');
        if strlength(pair) == 0, pair = "none reported"; end
        if isnan(c.collisionFree)
            line = 'Self-collision: UNKNOWN (geometry unavailable)';
            color = [0.55 0.35 0.07];
        elseif c.collisionFree && isfield(robot.collision,'minimumClearance') && ...
                c.clearance_m < robot.collision.minimumClearance
            line = sprintf('FREE but within %.0f mm margin | %.1f mm | %s', ...
                1000*robot.collision.minimumClearance, ...
                1000*c.clearance_m,pair);
            color = [0.62 0.35 0.04];
        elseif c.collisionFree
            line = sprintf('Self-collision: FREE | clearance %.1f mm | %s', ...
                1000*c.clearance_m,pair);
            color = [0.04 0.42 0.22];
        else
            line = sprintf('SELF-COLLISION | pair %s',pair);
            color = [0.75 0.12 0.10];
        end
        current = getappdata(fig,'WorkspaceRobotConfiguration');
        if numel(current) == numel(c.q) && max(abs(current-c.q)) > 1e-8
            poseLine = 'Displayed robot differs; press Reach.';
        else
            poseLine = 'Displayed robot is at the selected configuration.';
        end
        if strlength(message) > 0, poseLine = char(message); end
        statusText.String = {line,poseLine};
        statusText.ForegroundColor = color;
    end

    function reach()
        if selectedIndex == 0, return; end
        c = candidates(selectedIndex);
        [free,clearance,pair] = checkCandidateCollisions(c.q);
        candidates(selectedIndex).collisionFree = free(1);
        candidates(selectedIndex).clearance_m = clearance(1);
        candidates(selectedIndex).closestPair = pair(1,:);
        setRobotPose(c.q);
        if free(1) == 0
            message = "Colliding pose shown; no motion/path was checked.";
        elseif isnan(free(1))
            message = "Pose shown; collision geometry is unavailable.";
        else
            message = "Pose shown; only this endpoint was checked.";
        end
        refreshList();
    end

    function findAlternatives()
        if selectedIndex == 0, return; end
        findButton.Enable = 'off';
        drawnow;
        target = candidates(selectedIndex).transform;
        selectedQ = candidates(selectedIndex).q;
        try
            result = robot.solveIK(target,struct('qSeed',selectedQ, ...
                'numStarts',24,'seed',1));
            added = 0;
            if result.valid
                limits = robot.params.joints.positionLimits;
                for k = 1:size(result.q,1)
                    q = result.q(k,:);
                    if any(~isfinite(q)) || ...
                            any(q < limits(:,1).'-1e-9 | q > limits(:,2).'+1e-9)
                        continue;
                    end
                    if any(arrayfun(@(c) norm(c.q-q) < 1e-5,candidates))
                        continue;
                    end
                    T = getTransform(robot.model,q, ...
                        char(robot.structure.frames.endEffector));
                    dp = norm(T(1:3,4)-target(1:3,4));
                    R = target(1:3,1:3).'*T(1:3,1:3);
                    dr = acos(max(-1,min(1,(trace(R)-1)/2)));
                    if dp > 1e-5 || dr > 1e-4
                        continue;
                    end
                    [free,clearance,pair] = checkCandidateCollisions(q);
                    icn = linearICN(q);
                    accepted = free(1) == 1;
                    if data.analysisType == "usable"
                        accepted = accepted && ...
                            icn >= data.settings.manipulabilityThreshold;
                    elseif data.analysisType == "reachable"
                        accepted = true;
                    end
                    candidates(end+1) = makeCandidate("new IK",0,q,T, ...
                        icn,free(1),clearance(1),pair(1,:),accepted); %#ok<AGROW>
                    added = added+1;
                end
            end
            if added == 0
                message = "No additional IK solution found; original candidates remain.";
            else
                message = sprintf('%d new IK pose(s); graph statistics unchanged.',added);
                oldQ = candidates(selectedIndex).q;
                rankCandidates();
                selectedIndex = find(arrayfun( ...
                    @(c) norm(c.q-oldQ) < 1e-9,candidates),1);
            end
        catch ME
            message = "IK search failed: " + string(ME.message);
        end
        refreshList();
    end

    function state = getState()
        state.clickedSampleIndex = clickedSampleIndex;
        state.selectedVoxel = selectedVoxel;
        state.candidates = candidates;
        state.selectedIndex = selectedIndex;
        state.message = message;
        state.robotConfiguration = getappdata(fig,'WorkspaceRobotConfiguration');
    end
end

function candidates = emptyCandidates(dof)
template = struct('source',"",'sampleIndex',0,'q',zeros(1,dof), ...
    'transform',eye(4),'point',zeros(1,3),'icn',NaN, ...
    'collisionFree',NaN,'clearance_m',NaN, ...
    'closestPair',strings(1,2),'acceptedByPlot',false);
candidates = repmat(template,1,0);
end
