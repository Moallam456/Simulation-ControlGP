function summary = verifyPathProgressVelocityExecution(observation)
%VERIFYPATHPROGRESSVELOCITYEXECUTION Run the ideal kinematic simulation sweep.
if nargin<1, observation = 1; end
validateattributes(observation,{'double'},{'scalar','finite','positive'});
root = fileparts(fileparts(mfilename('fullpath')));
trajectory = fileparts(root);
output = fullfile(root,'results','velocity_execution');
if ~isfolder(output), mkdir(output); end
oldPath = path;
restore = onCleanup(@() path(oldPath));
addpath(fullfile(root,'tests'),fullfile(root,'helpers'),fullfile(trajectory,'Pipeline'));
verifyExecutor();
fprintf('Constant command, partial interval, and discrete feedback checks passed.\n');

names = ["line","arc_exact_length","fullcircle_exact_length","multi_scurve"];
saved = load(fullfile(root,'results','path_progress_after.mat'),'after');
prior = load(fullfile(root,'results','exact_geometry_comparison','exact_geometry_runs.mat'),'runs');
multi = load(fullfile(root,'results','path_progress_multi_scurve.mat'),'scurve');
retimed = table();
robot = loadTrajectoryRobot();
frames = packFrames(robot);
limits = robot.params.joints.positionLimits;
for k = 1:numel(names)
    name = names(k);
    switch name
        case "line", data{k} = saved.after.line; %#ok<AGROW>
        case "multi_scurve", data{k} = multi.scurve;
        otherwise
            data{k} = pathProgressRunDemo(name);
            originalName = erase(name,"_exact_length");
            old = prior.runs.(originalName).exact;
            retimed = [retimed;timingComparison(name,old,data{k})]; %#ok<AGROW>
    end
    d = data{k};
    reference{k} = pathProgressPrepareReference(d); %#ok<AGROW>
    [q,qd,qdd,timing] = pathProgressEvaluateReference(reference{k},d.tGlobal);
    assert(max(abs(q-d.qPath),[],'all')<=1e-12 && max(abs(qd-d.qDot),[],'all')<=1e-11 && ...
        max(abs(qdd-d.qDDot),[],'all')<=1e-10,'Continuous evaluator differs at the reference samples.');
    validateTiming(reference{k},d,timing);
    verifyBatch(robot,frames,d);
    phaseTimes = [];
    for i = 1:numel(reference{k}.segment)
        phaseTimes = [phaseTimes,reference{k}.edges(i)+reference{k}.segment(i).phase(:,1).']; %#ok<AGROW>
    end
    desired{k}.phaseTimes = phaseTimes; %#ok<AGROW>
end
writetable(retimed,fullfile(output,'timing_cleanup.csv'));
fprintf('Exact-length timing, saved sample agreement, and Toolbox FK/Jacobian cross-checks passed.\n');

periods = [1 2 5 10 20 50]/1000;
gains = [0 2 8];                  % s^-1, ideal simulation only
offset = deg2rad([.20;-.15;.10;-.20;.12;-.10]);
inits = [zeros(6,1),offset];
grid = .0005;
assert(max(gains)*max(periods)<1,'The tested proportional error recurrence must be nonoscillatory.');
waypoints = table();
rows = cell(1,numel(data)*numel(periods)*numel(gains)*size(inits,2));
rowIndex = 0;
traces = struct();
for k = 1:numel(data)
    d = data{k};
    vlim = d.rateLimits.velocityURDF;
    q0 = pathProgressEvaluateReference(reference{k},0);
    assert(all(q0+inits>=limits(:,1) & q0+inits<=limits(:,2),'all'), ...
        'The fixed initial joint offset is outside the model limits.');
    for period = periods
        for gain = gains
            for i = 1:size(inits,2)
                args = {d,reference{k},frames,desired{k},period,gain,inits(:,i),observation,grid,limits,vlim};
                if any(k==[3 4]) && period==periods(end) && any(gain==[0 8]) && i==2
                    [row,stops,trace] = pathProgressExecutionMetrics(args{:});
                    key = char(d.demo+"_kp"+gain);
                    traces.(key) = trace;
                else
                    [row,stops] = pathProgressExecutionMetrics(args{:});
                end
                rowIndex = rowIndex+1;
                rows{rowIndex} = row;
                waypoints = [waypoints;stops]; %#ok<AGROW>
            end
        end
        fprintf('%s | %2d ms: six cases measured.\n',d.demo,round(period*1000));
    end
end
summary = struct2table([rows{:}]);
writetable(summary,fullfile(output,'summary.csv'));
writetable(waypoints,fullfile(output,'waypoint_stops.csv'));

[~,worstPosition] = max(summary.PeakTcp_m);
[~,worstSpeed] = max(summary.PeakSpeedError_m_s);
fast = find(summary.Period_ms==1 & summary.Kp_per_s==8 & summary.Initial=="Offset");
[~,idx] = max(summary.PeakTcp_m(fast));
convergence = summary([],:);
for i = unique([worstPosition,worstSpeed,fast(idx)])
    r = summary(i,:);
    k = find(names==r.Case,1);
    ini = inits(:,1+(r.Initial=="Offset"));
    convergence = [convergence;r]; %#ok<AGROW>
    for h = [grid/2,grid/4]
        fine = pathProgressExecutionMetrics(data{k},reference{k},frames,desired{k}, ...
            r.Period_ms/1000,r.Kp_per_s,ini,observation,h,limits,data{k}.rateLimits.velocityURDF);
        convergence = [convergence;struct2table(fine)]; %#ok<AGROW>
    end
end
writetable(convergence,fullfile(output,'grid_convergence.csv'));
save(fullfile(output,'execution.mat'),'summary','waypoints','convergence','retimed', ...
    'data','reference','traces','periods','gains','offset','observation','grid','frames','limits');
pathProgressPlotVelocityExecution(output,summary,traces);
pathProgressWriteExecutionReport(output,summary,waypoints,convergence,retimed,observation,offset);
fprintf('Velocity execution evidence saved for %d cases.\n',height(summary));
end

function timing = timingComparison(name,old,new)
a = old.segments(1);
b = new.segments(1);
assert(isequal(a.exactCircle,b.exactCircle) && isequal(old.qStart,new.qStart) && ...
    isequal(a.startRotation,b.startRotation) && isequal(a.endRotation,b.endRotation) && ...
    a.orientationSweep==b.orientationSweep && a.profile.dt==b.profile.dt, ...
    'Exact-length timing changed geometry, start, orientation or sample setting.');
physicalOld = [a.profile.requestedVMax,a.profile.requestedAMax,a.profile.requestedJMax]*a.length;
physicalNew = [b.profile.requestedVMax,b.profile.requestedAMax,b.profile.requestedJMax]*b.exactLength;
assert(max(abs(physicalNew-physicalOld))<=1e-12,'TCP motion settings changed.');
timing = table(name,a.length,b.exactLength,old.tGlobal(end),new.tGlobal(end), ...
    new.tGlobal(end)-old.tGlobal(end),numel(old.tGlobal),numel(new.tGlobal), ...
    max(old.tcpSpeedPath),max(new.tcpSpeedPath), ...
    'VariableNames',{'Case','PriorTimingLength_m','ExactTimingLength_m','PriorDuration_s', ...
    'ExactDuration_s','DurationDelta_s','PriorSamples','ExactSamples','PriorPeakSpeed_m_s','ExactPeakSpeed_m_s'});
end

function verifyExecutor()
ticks = [0,.1,.2,.25,.35];
target = zeros(size(ticks));
[x,v] = pathProgressExecuteZoh(ticks,target,[2,2,2,0,0],1,0);
[q,held,left] = pathProgressObserveZoh(ticks,x,v,[0,.07,.23,.25,.29,.35]);
assert(max(abs(q-[1,1.14,1.46,1.5,1.5,1.5]))<1e-14 && held(4)==0 && left(4)==2);
t = (0:10)*.05;
[x,~] = pathProgressExecuteZoh(t,zeros(size(t)),zeros(size(t)),.2,8);
assert(max(abs(x-.2*(1-8*.05).^(0:10)))<1e-14);
end

function validateTiming(reference,data,timing)
assert(abs(reference.edges(end)-data.tGlobal(end))<=1e-12);
for k = 1:numel(reference.segment)
    fit = data.fit(k);
    expected = data.segments(k).timing(2:4,end-numel(fit.globalIndices)+1:end);
    assert(max(abs(timing(2:4,fit.globalIndices)-expected),[],'all')<=1e-12);
end
[~,v,a] = pathProgressEvaluateReference(reference,reference.edges);
assert(all(v==0,'all') && all(a==0,'all'),'A reference waypoint does not stop.');
end

function frames = packFrames(robot)
q = zeros(1,robot.structure.dof);
tcp = char(robot.structure.frames.endEffector);
J = geometricJacobian(robot.model,q,tcp);
previous = eye(4);
for j = 1:numel(q)
    current = getTransform(robot.model,q,char(robot.structure.bodyNames(j)));
    frames.fixed(:,:,j) = previous\current;
    u = current(1:3,1:3).'*J(1:3,j);
    frames.axis(:,j) = u;
    frames.skew(:,:,j) = [0 -u(3) u(2);u(3) 0 -u(1);-u(2) u(1) 0];
    previous = current;
end
frames.tool = previous\getTransform(robot.model,q,tcp);
end

function verifyBatch(robot,frames,data)
tcp = char(robot.structure.frames.endEffector);
indices = unique(round(linspace(1,numel(data.tGlobal),11)));
for i = indices
    q = data.qPath(:,i)+deg2rad([.2;-.1;.05;0;-.12;.05]);
    T = getTransform(robot.model,q.',tcp);
    J = geometricJacobian(robot.model,q.',tcp);
    [p,R,speed,leftSpeed] = pathProgressBatchTcp(frames,repmat(q,1,6),eye(6),-eye(6));
    assert(max(abs(p-T(1:3,4)),[],'all')<=1e-12 && ...
        max(abs(R-T(1:3,1:3)),[],'all')<=1e-12 && ...
        max(abs(speed-vecnorm(J(4:6,:),2,1)))<=1e-12 && ...
        max(abs(leftSpeed-speed))<=1e-12,'Batch model disagrees with Toolbox FK/Jacobian.');
    v = [1;-2;3;-4;5;-6]/10;
    [~,~,mixed] = pathProgressBatchTcp(frames,q,v,v);
    assert(abs(mixed-norm(J(4:6,:)*v))<=1e-12);
end
end
