function result = optimizeParkConfiguration(robot,settings)
% OPTIMIZEPARKCONFIGURATION Practical minimax holding-load park candidates.
% Analysis settings/UI are in park_position_analysis. No model is modified.
if nargin<2, settings=struct(); end
defaults = struct('gravity',[0 0 -9.81],'numSamples',1000,'numStarts',4, ...
    'seed',23,'minimumClearance',.010,'mountingPlaneZ_m',0, ...
    'planeTolerance_m',1e-6,'jointStopMargin_deg',5, ...
    'allowableHoldingTorque_Nm',[],'maximumHeight_m',[], ...
    'maximumRadius_m',[],'localSpread_deg',[],'maxSeedCollisionChecks',60, ...
    'maxFunctionEvaluations',350,'maxIterations',60,'numCandidates',4, ...
    'maxCollisionRefinements',2,'patternRefinements',1, ...
    'pathStep_deg',2,'checkPaths',true,'useRRT',false,'rrtIterations',100, ...
    'maxRRTAttempts',1, ...
    'showProgress',false);
for name=string(fieldnames(defaults)).'
    if ~isfield(settings,name), settings.(name)=defaults.(name); end
end
for name=["numSamples" "numStarts" "maxFunctionEvaluations" "maxIterations" ...
        "numCandidates" "maxSeedCollisionChecks" "rrtIterations"]
    validateattributes(settings.(name),{'numeric'},{'scalar','integer','positive'});
end
for name=["minimumClearance" "planeTolerance_m" "jointStopMargin_deg"]
    validateattributes(settings.(name),{'numeric'},{'scalar','finite','nonnegative'});
end
validateattributes(settings.seed,{'numeric'},{'scalar','integer','nonnegative'});
validateattributes(settings.pathStep_deg,{'numeric'},{'scalar','finite','positive'});
validateattributes(settings.mountingPlaneZ_m,{'numeric'},{'scalar','real','finite'});
previousRng=rng; cleanup=onCleanup(@() rng(previousRng));
rng(settings.seed,'twister'); started=tic;
[model,validation]=prepareDynamicsModel(robot,settings.gravity);
n=robot.structure.dof; limits=robot.params.joints.positionLimits;
span=diff(limits,1,2).'; lower=limits(:,1).'; upper=limits(:,2).';
margin=min(deg2rad(settings.jointStopMargin_deg),span/10);
lo=lower+margin; hi=upper-margin; range=hi-lo;
home=reshape(robot.params.joints.homePosition,1,[]);
validateattributes(home,{'numeric'},{'row','numel',n,'finite'});
assert(all(home>=lower & home<=upper),'park:InvalidHome','Home is outside joint limits.');
catalog=struct();
if isfield(robot.params,'park'), catalog=robot.params.park; end
if isempty(settings.localSpread_deg)
    settings.localSpread_deg=25*ones(1,n);
    if isfield(catalog,'localSpread_deg'), settings.localSpread_deg=catalog.localSpread_deg; end
end
if isscalar(settings.localSpread_deg), settings.localSpread_deg=repmat(settings.localSpread_deg,1,n); end
validateattributes(settings.localSpread_deg,{'numeric'},{'row','numel',n,'nonnegative','finite'});
for name=["maxCollisionRefinements" "patternRefinements" "maxRRTAttempts"]
    validateattributes(settings.(name),{'numeric'},{'scalar','integer','nonnegative'});
end
for name=["maximumHeight_m" "maximumRadius_m"]
    if isempty(settings.(name))
        if isfield(catalog,name), settings.(name)=catalog.(name);
        else, settings.(name)=Inf; end
    end
    validateattributes(settings.(name),{'numeric'},{'scalar','positive','real','nonnan'});
end
ratings=settings.allowableHoldingTorque_Nm;
if isempty(ratings), scales=ones(1,n); normalized=false;
else
    validateattributes(ratings,{'numeric'},{'vector','numel',n,'finite','positive','real'});
    ratings=reshape(ratings,1,[]); scales=ratings; normalized=true;
end
geometry=bodyGeometry(model);
cache=containers.Map('KeyType','char','ValueType','any');
collisionCache=containers.Map('KeyType','char','ValueType','any');
bank=struct([]); trials=struct('profile',{},'exitflag',{},'evaluations',{});
reference=pose(home,"configured home");
seeds=max(lo,min(hi,home));
if isfield(catalog,'seedConfigurations')
    supplied=catalog.seedConfigurations;
    validateattributes(supplied,{'numeric'},{'2d','ncols',n,'finite'});
    seeds=[seeds;max(lo,min(hi,supplied))];
end
% Always check the deliberate practical seeds. Low-torque colliding random
% samples must not exhaust the screening budget before these are considered.
for k=1:size(seeds,1), accept(seeds(k,:),"robot-specific practical seed"); end
seedBankCount=numel(bank);
% Deterministic Sobol coverage, with a fallback for installations without it.
if exist('sobolset','file')==2
    unit=net(scramble(sobolset(n,'Skip',1024),'MatousekAffineOwen'),settings.numSamples);
    samplingMethod="scrambled Sobol";
else
    unit=rand(settings.numSamples,n); samplingMethod="seeded pseudorandom";
end
local=seeds(1+mod((0:settings.numSamples-1).',size(seeds,1)),:) + ...
    (2*unit-1).*deg2rad(settings.localSpread_deg);
samples=unique([seeds;max(lo,min(hi,local));lo+unit.*range],'rows','stable');
score=Inf(size(samples,1),1); compact=score; eligible=false(size(score));
progress('Screening %d local/global samples for geometry and holding load...',size(samples,1));
for k=1:size(samples,1)
    s=state(samples(k,:));
    eligible(k)=practical(s);
    if eligible(k)
        score(k)=max(abs(s.torque_Nm)./scales); compact(k)=s.diagonal_m;
    end
end
indices=find(eligible); [~,a]=sort(score(indices)); [~,b]=sort(compact(indices));
order=unique(reshape([indices(a).';indices(b).'],[],1),'stable');
checks=0;
for i=order.'
    if checks>=settings.maxSeedCollisionChecks, break; end
    if ~isempty(bank) && any(arrayfun(@(p) norm((p.q-samples(i,:))./span)<.06,bank)), continue; end
    checks=checks+1; accept(samples(i,:),"sampled");
    if numel(bank)>=settings.numStarts*3, break; end
end
canOptimize=exist('fmincon','file')==2 && license('test','Optimization_Toolbox');
active="balanced"; loadCap=Inf; starts=zeros(0,n); collisionRepairs=0; rrtAttempts=0;
homeAccessibleSeedIndex=0;
if settings.checkPaths && seedBankCount>0 && reference.plane.abovePlane && reference.collision.hasRequiredClearance
    [~,seedOrder]=sort([bank(1:seedBankCount).objective]);
    progress('Screening practical seed approaches from configured home...');
    for seedIndex=seedOrder
        seedRoute=checkRoute(interpolated([home;bank(seedIndex).q],settings.pathStep_deg));
        if seedRoute.passed, homeAccessibleSeedIndex=seedIndex; break; end
    end
end
if ~isempty(bank)
    [~,ranked]=sort([bank.objective]);
    for i=ranked
        x=(bank(i).q-lo)./range;
        if isempty(starts) || all(vecnorm(starts-x,2,2)>.08), starts(end+1,:)=x; end %#ok<AGROW>
        if size(starts,1)>=settings.numStarts, break; end
    end
    if homeAccessibleSeedIndex>0
        accessibleX=(bank(homeAccessibleSeedIndex).q-lo)./range;
        starts=unique([accessibleX;starts],'rows','stable');
        starts=starts(1:min(settings.numStarts,size(starts,1)),:);
    end
end
if canOptimize && ~isempty(starts)
    options=optimoptions('fmincon','Algorithm','sqp','Display','off', ...
        'MaxFunctionEvaluations',settings.maxFunctionEvaluations, ...
        'MaxIterations',settings.maxIterations,'ConstraintTolerance',1e-7, ...
        'OptimalityTolerance',1e-8,'StepTolerance',1e-9,'OutputFcn',@keepIteration);
    for k=1:size(starts,1)
        progress('Minimax holding-load refinement %d / %d...',k,size(starts,1));
        refine(starts(k,:),options);
    end
    if exist('patternsearch','file')==2 && license('test','GADS_Toolbox')
        pollOptions=optimoptions('patternsearch','Display','off','InitialMeshSize',.025, ...
            'MeshTolerance',1e-4,'MaxIterations',60, ...
            'MaxFunctionEvaluations',min(240,settings.maxFunctionEvaluations));
        for pollIndex=1:min(settings.patternRefinements,size(starts,1))
            progress('Feasible mesh-poll refinement %d...',pollIndex);
            [~,bestIndex]=min([bank.objective]);
            pollStart=(bank(bestIndex).q-lo)./range;
            [pollX,~,flag,out]=patternsearch(@feasibleObjective,pollStart,[],[],[],[], ...
                zeros(1,n),ones(1,n),[],pollOptions);
            accept(lo+pollX.*range,"feasible mesh-poll refinement");
            trials(end+1)=struct('profile',"feasible patternsearch",'exitflag',flag,'evaluations',out.funccount); %#ok<AGROW>
        end
    end
    [~,bestIndex]=min([bank.objective]);
    % Bounded load trade-offs: minimizing one joint cannot overload the others.
    loadCap=max([bank(bestIndex).objective]*1.5,bank(bestIndex).objective+min(1,1/max(scales)));
    for profile=["compact" "j2" "j3"]
        if (profile=="j2" && n<2) || (profile=="j3" && n<3), continue; end
        active=profile;
        progress('Refining %s alternative under the peak-load cap...',profile);
        refine((bank(bestIndex).q-lo)./range,options);
    end
end
% Select meaningful, geometrically distinct alternatives, not duplicate rows.
if homeAccessibleSeedIndex>0
    accessibleSeed=bank(homeAccessibleSeedIndex).q;
    nearby=find(arrayfun(@(entry) norm((entry.q-accessibleSeed)./span)<.35,bank));
    [~,nearOrder]=sort([bank(nearby).objective]);
    for nearIndex=reshape(nearby(nearOrder(1:min(8,numel(nearOrder)))) ,1,[])
        checkedApproach=checkRoute(interpolated([home;bank(nearIndex).q],settings.pathStep_deg));
        if checkedApproach.passed, homeAccessibleSeedIndex=nearIndex; break; end
    end
end
candidates=struct('id',{},'name',{},'pose',{},'isStableCandidate',{},'isPareto',{}, ...
    'paths',{},'returnVerified',{});
if ~isempty(bank)
    loads=[bank.objective].'; sizes=arrayfun(@(p) p.plane.diagonal_m,bank).';
    frontier=true(numel(bank),1);
    for i=1:numel(bank)
        frontier(i)=~any(loads<=loads(i)+1e-8 & sizes<=sizes(i)+1e-8 & ...
            (loads<loads(i)-1e-8 | sizes<sizes(i)-1e-8));
    end
    [~,i]=min(loads); add("balanced","Lowest peak holding load",i);
    allowed=find(loads<=loadCap+1e-6);
    if isempty(allowed), allowed=(1:numel(bank)).'; end
    [~,j]=min(sizes(allowed)); add("compact","Compact park (bounded holding load)",allowed(j));
    % Retain a home-accessible seed even if disconnected static poses dominate
    % both load and size. It still needs checks from the other work starts.
    if homeAccessibleSeedIndex>0
        add("home_accessible","Upright / folded return candidate",homeAccessibleSeedIndex);
    end
    for joint=2:min(3,n)
        [~,j]=min(arrayfun(@(p) abs(p.torque_Nm(joint)),bank(allowed)));
        add("j"+joint,"Lowest J"+joint+" (bounded peak load)",allowed(j));
    end
    [~,idx]=sort(loads);
    for i=idx.'
        if frontier(i), add("pareto_"+i,"Load / compactness alternative",i); end
        if numel(candidates)>=settings.numCandidates, break; end
    end
end
workingStarts=workingConfigurations(robot,home);
startEvidence=struct('q',{},'valid',{},'reason',{});
for k=1:size(workingStarts,1)
    q=workingStarts(k,:); s=state(q); valid=all(q>=lower & q<=upper) && s.abovePlane;
    reason="Below mounting plane or outside joint limits";
    if valid
        c=collisionAt(q); valid=c.hasRequiredClearance; reason="Insufficient self-clearance";
    end
    if valid, reason="Valid start"; end
    startEvidence(end+1)=struct('q',q,'valid',logical(valid),'reason',reason); %#ok<AGROW>
end
if settings.checkPaths
    for k=1:numel(candidates)
        progress('Checking return routes for %s...',candidates(k).name);
        routes=struct('startIndex',{},'passed',{},'method',{},'q',{},'minimumClearance_m',{}, ...
            'minimumZ_m',{},'maxStep_deg',{},'status',{});
        for j=find([startEvidence.valid])
            route=returnRoute(startEvidence(j).q,candidates(k).pose.q);
            route.startIndex=j; routes(end+1)=route; %#ok<AGROW>
        end
        candidates(k).paths=routes;
        candidates(k).returnVerified=~isempty(routes) && all([routes.passed]);
    end
    if ~isempty(candidates)
        % Prefer candidates reached from every valid representative start.
        [~,routeOrder]=sortrows([~[candidates.returnVerified].' ...
            arrayfun(@(entry) entry.pose.objective,candidates).'],[1 2]);
        candidates=candidates(routeOrder);
    end
end
% Curvature is a diagnostic, not a hard constraint that forces a horizontal arm.
for k=1:numel(candidates)
    finalPose=candidates(k).pose;
    finalPose.collision=analyzeCollisions(robot,finalPose.q,struct('minimumClearance',settings.minimumClearance));
    assert(finalPose.collision.hasRequiredClearance && practical(state(finalPose.q)));
    finalPose.stability=curvature(model,finalPose.q,limits,finalPose.torque_Nm);
    candidates(k).pose=finalPose; candidates(k).isStableCandidate=finalPose.stability.passiveEquilibrium;
end
result.settings=settings; result.found=~isempty(candidates); result.candidates=candidates;
result.best=[]; result.reference=reference; result.bank=bank; result.trials=trials;
result.returnVerified=~isempty(candidates) && candidates(1).returnVerified;
result.workingStarts=startEvidence; result.optimizationAvailable=logical(canOptimize);
result.optimizationPerformed=~isempty(trials); result.status="NO_FEASIBLE_SAMPLED_POSE";
result.stableFound=false;
if result.found
    result.best=candidates(1).pose; result.status="PRACTICAL_PARK_CANDIDATES_FOUND";
    result.stableFound=any([candidates.isStableCandidate]);
    if settings.checkPaths && ~result.returnVerified, result.status="POSES_FOUND_BUT_RETURN_NOT_VERIFIED"; end
end
result.meta=dynamicsMetadata(robot,model.Gravity,validation,"practical park configuration search");
result.meta.globalMinimumCertified=false; result.meta.motorRatingsAvailable=normalized;
result.meta.objective="min max(abs(gravityTorque)) [N m]";
if normalized, result.meta.objective="min max(abs(gravityTorque)./allowableHoldingTorque)"; end
result.meta.samplingMethod=samplingMethod;
result.meta.alternativePeakLoadCap=loadCap;
result.meta.scope="Modeled self-clearance and whole visual geometry above mounting plane. Return routes are sampled checks, not continuous certification. No workcell/cable/brake verification.";
result.meta.references=[ ...
    "https://knowledge.motoman.com/hc/en-us/articles/33649986738834-Shipping-Position"
    "https://knowledge.motoman.com/hc/en-us/articles/4415152176791-Home-Position-Second-Home-Position-and-Work-Home-Position"];
result.statistics=struct('elapsedSeconds',toc(started),'collisionEvaluations',collisionCache.Count, ...
    'sampleCount',size(samples,1),'geometryEligibleSamples',nnz(eligible), ...
    'localSolves',numel(trials),'rrtAttempts',rrtAttempts);
result.samples=struct('q',samples,'objective',score,'geometryEligible',eligible);

    function s=state(q)
        key=poseKey(q);
        if isKey(cache,key), s=cache(key); return; end
        minimum=Inf(1,3); maximum=-Inf(1,3); heights=Inf(numel(geometry),1); radius=0;
        weighted=zeros(1,3); mass=0; energy=0;
        for bodyIndex=1:numel(geometry)
            g=geometry(bodyIndex); T=eye(4);
            if ~g.isBase, T=getTransform(model,q,g.name); end
            if ~isempty(g.vertices)
                P=g.vertices*T(1:3,1:3).'+T(1:3,4).';
                minimum=min(minimum,min(P,[],1)); maximum=max(maximum,max(P,[],1));
                heights(bodyIndex)=min(P(:,3));
                radius=max(radius,max(vecnorm(P(:,1:2),2,2)));
            end
            center=(T(1:3,1:3)*g.com.'+T(1:3,4)).';
            weighted=weighted+g.mass*center; mass=mass+g.mass;
            energy=energy-g.mass*dot(model.Gravity,center);
        end
        extent=maximum-minimum;
        s=struct('energy_J',energy,'com_m',weighted/max(mass,eps), ...
            'bodyMinimumZ_m',heights,'minimumZ_m',minimum(3), ...
            'abovePlane',minimum(3)>=settings.mountingPlaneZ_m-settings.planeTolerance_m, ...
            'maximumZ_m',maximum(3),'extent_m',extent,'diagonal_m',norm(extent), ...
            'radius_m',radius, ...
            'torque_Nm',reshape(gravityTorque(model,q),1,[]));
        cache(key)=s;
    end

    function yes=practical(s)
        yes=s.abovePlane && s.maximumZ_m<=settings.mountingPlaneZ_m+settings.maximumHeight_m+1e-7 && ...
            s.radius_m<=settings.maximumRadius_m+1e-7;
    end

    function c=collisionAt(q)
        key=poseKey(q);
        if isKey(collisionCache,key), c=collisionCache(key); return; end
        c=analyzeCollisions(robot,q,struct('minimumClearance',settings.minimumClearance));
        collisionCache(key)=c;
    end

    function evaluatedPose=pose(q,source)
        s=state(q); t=s.torque_Nm; utilization=NaN(1,n);
        if normalized, utilization=abs(t)./scales; end
        evaluatedPose=struct('q',q,'torque_Nm',t,'objective',max(abs(t)./scales), ...
            'torqueNorm_Nm',norm(t),'peakAbsTorque_Nm',max(abs(t)), ...
            'utilization',utilization,'collision',collisionAt(q),'plane',s, ...
            'energy_J',s.energy_J,'source',source,'stability',curvature(model,q,limits,t));
    end

    function ok=accept(q,source)
        ok=false; s=state(q);
        if ~practical(s) || any(q<lo-1e-8 | q>hi+1e-8), return; end
        c=collisionAt(q); if ~c.hasRequiredClearance, return; end
        ok=true;
        if isempty(bank) || all(arrayfun(@(p) norm((p.q-q)./span)>1e-7,bank))
            bank=[bank pose(q,source)];
        end
    end

    function refine(x0,options)
        s=state(lo+x0.*range); z0=[x0 max(abs(s.torque_Nm)./scales)];
        [z,~,flag,out]=fmincon(@objective,z0,[],[],[],[], ...
            [zeros(1,n) 0],[ones(1,n) Inf],@(z) constraints(z,false),options);
        refinedQ=lo+z(1:n).*range;
        valid=withinActiveLoadBudget(refinedQ) && accept(refinedQ,active+" refinement");
        trials(end+1)=struct('profile',active,'exitflag',flag,'evaluations',out.funcCount);
        if ~valid && practical(state(lo+z(1:n).*range)) && collisionRepairs<settings.maxCollisionRefinements
            collisionRepairs=collisionRepairs+1;
            repair=optimoptions(options,'MaxFunctionEvaluations',min(160,settings.maxFunctionEvaluations));
            [z,~,flag,out]=fmincon(@objective,z0,[],[],[],[], ...
                [zeros(1,n) 0],[ones(1,n) Inf],@(z) constraints(z,true),repair);
            refinedQ=lo+z(1:n).*range;
            if withinActiveLoadBudget(refinedQ), accept(refinedQ,active+" clearance refinement"); end
            trials(end+1)=struct('profile',active+" clearance",'exitflag',flag,'evaluations',out.funcCount);
        end
    end

    function yes=withinActiveLoadBudget(q)
        evidence=state(q);
        yes=active=="balanced" || max(abs(evidence.torque_Nm)./scales)<=loadCap+1e-6;
    end

    function value=feasibleObjective(x)
        pollQ=lo+x.*range; pollState=state(pollQ);
        value=1e6+max(abs(pollState.torque_Nm)./scales);
        if ~practical(pollState), return; end
        pollCollision=collisionAt(pollQ);
        if ~pollCollision.hasRequiredClearance, return; end
        value=max(abs(pollState.torque_Nm)./scales)+1e-5*pollState.diagonal_m+1e-5*sum((pollQ-home).^2);
        if max(abs(pollState.torque_Nm)./scales)<min([bank.objective])-1e-5
            accept(pollQ,"feasible patternsearch improvement");
        end
    end

    function f=objective(z)
        s=state(lo+z(1:n).*range);
        switch active
            case "compact", f=s.diagonal_m;
            case "j2", f=(s.torque_Nm(2)/scales(2))^2;
            case "j3", f=(s.torque_Nm(3)/scales(3))^2;
            otherwise, f=z(end);
        end
        % Only a very small tie-break; reported load stays an actual minimax.
        f=f+1e-5*sum((lo+z(1:n).*range-home).^2)+1e-5*s.diagonal_m;
    end

    function stop=keepIteration(z,info,stage)
        stop=false;
        if ~strcmp(stage,'iter') || mod(info.iteration,3)~=0, return; end
        q=lo+z(1:n).*range; s=state(q);
        if ~practical(s), return; end
        load=max(abs(s.torque_Nm)./scales);
        if active=="balanced"
            improved=load<min([bank.objective])-1e-5;
        elseif active=="compact"
            improved=load<=loadCap+1e-7 && s.diagonal_m<min(arrayfun(@(p) p.plane.diagonal_m,bank))-1e-5;
        else
            targetJoint=str2double(extractAfter(active,'j'));
            improved=load<=loadCap+1e-7 && abs(s.torque_Nm(targetJoint))<min(arrayfun(@(p) abs(p.torque_Nm(targetJoint)),bank))-1e-5;
        end
        if improved, accept(q,active+" accepted intermediate"); end
    end

    function [c,ceq]=constraints(z,withCollision)
        q=lo+z(1:n).*range; s=state(q); t=s.torque_Nm./scales;
        c=[t(:)-z(end);-t(:)-z(end);settings.mountingPlaneZ_m-s.bodyMinimumZ_m];
        if isfinite(settings.maximumHeight_m)
            c(end+1)=s.maximumZ_m-settings.mountingPlaneZ_m-settings.maximumHeight_m;
        end
        if isfinite(settings.maximumRadius_m), c(end+1)=s.radius_m-settings.maximumRadius_m; end
        if active~="balanced", c(end+1)=z(end)-loadCap; end
        if withCollision
            evidence=collisionAt(q); d=min(evidence.minimumClearance,.1);
            if evidence.isSelfColliding, d=-1e-4; end
            c(end+1)=(settings.minimumClearance+1e-6-d)/.1;
        end
        c(~isfinite(c))=-1; ceq=[];
    end

    function add(id,name,index)
        if numel(candidates)>=settings.numCandidates, return; end
        selectedPose=bank(index);
        if ~isempty(candidates) && any(arrayfun(@(c) max(abs(c.pose.q-selectedPose.q))<deg2rad(3),candidates)), return; end
        candidates(end+1)=struct('id',id,'name',name,'pose',selectedPose, ...
            'isStableCandidate',false,'isPareto',frontier(index),'paths',struct([]), ...
            'returnVerified',false);
    end

    function route=returnRoute(start,goal)
        route=checkRoute(interpolated([start;goal],settings.pathStep_deg));
        route.method="direct joint interpolation";
        if route.passed, return; end
        % A known reference waypoint is useful, but never assumed safe.
        viaHome=checkRoute(interpolated([start;home;goal],settings.pathStep_deg));
        viaHome.method="joint interpolation via configured home";
        if viaHome.passed, route=viaHome; return; end
        if ~settings.useRRT || rrtAttempts>=settings.maxRRTAttempts, return; end
        rrtAttempts=rrtAttempts+1;
        % Use MATLAB's planner; independently enforce plane and clearance after.
        % The plane participates in planning, not just in the final rejection.
        % Its box extends below the robot; the tiny gap avoids base contact.
        floorBox=collisionBox(10,10,10);
        floorBox.Pose=trvec2tform([0 0 settings.mountingPlaneZ_m-5-2*settings.planeTolerance_m]);
        planner=manipulatorRRT(model,{floorBox});
        planner.SkippedSelfCollisions=cellstr(reference.collision.settings.skippedSelfCollisionPairs);
        planner.ValidationDistance=deg2rad(settings.pathStep_deg);
        planner.MaxIterations=settings.rrtIterations;
        planner.MaxConnectionDistance=.3;
        [waypoints,info]=plan(planner,start,goal);
        if ~info.IsPathFound, route.status="No checked return route found"; return; end
        alternative=checkRoute(interpolated(waypoints,settings.pathStep_deg));
        alternative.method="MATLAB manipulatorRRT + independent geometry/clearance checks";
        if alternative.passed, route=alternative; end
    end

    function route=checkRoute(Q)
        minZ=Inf; clearance=Inf; passed=true;
        for routeIndex=1:size(Q,1)
            s=state(Q(routeIndex,:)); minZ=min(minZ,s.minimumZ_m);
            if ~s.abovePlane || any(Q(routeIndex,:)<lower | Q(routeIndex,:)>upper), passed=false; break; end
            c=collisionAt(Q(routeIndex,:)); clearance=min(clearance,c.minimumClearance);
            if ~c.hasRequiredClearance, passed=false; break; end
        end
        status="Sampled route passed";
        if ~passed, status="Route rejected; not a safe command"; end
        route=struct('startIndex',0,'passed',logical(passed),'method',"",'q',Q, ...
            'minimumClearance_m',clearance,'minimumZ_m',minZ, ...
            'maxStep_deg',settings.pathStep_deg,'status',status);
    end

    function progress(varargin)
        if settings.showProgress, fprintf('Park search: %s\n',sprintf(varargin{:})); end
    end
end

function Q=workingConfigurations(robot,home)
Q=home;
if isfield(robot,'dynamicsCases') && ~isempty(robot.dynamicsCases)
    for c=reshape(robot.dynamicsCases,1,[])
        if isfield(c,'enabled') && ~c.enabled, continue; end
        if isfield(c,'startConfiguration') && numel(c.startConfiguration)==numel(home)
            Q(end+1,:)=reshape(c.startConfiguration,1,[]); %#ok<AGROW>
        end
    end
end
Q=unique(Q,'rows','stable');
end

function Q=interpolated(waypoints,step)
Q=waypoints(1,:);
for k=2:size(waypoints,1)
    count=max(1,ceil(max(abs(waypoints(k,:)-waypoints(k-1,:)))/deg2rad(step)));
    Q=[Q;waypoints(k-1,:)+(1:count).'/count.*(waypoints(k,:)-waypoints(k-1,:))]; %#ok<AGROW>
end
end

function g=bodyGeometry(model)
bodies=[{model.Base} model.Bodies];
g=struct('name',{},'vertices',{},'mass',{},'com',{},'isBase',{});
for b=1:numel(bodies)
    body=bodies{b}; visuals=getVisual(body); P=zeros(0,3);
    for v=1:numel(visuals)
        m=visuals(v); p=m.Triangulation.Points.*m.Scale;
        P=[P;p*m.Tform(1:3,1:3).'+m.Tform(1:3,4).']; %#ok<AGROW>
    end
    if isempty(P) && ~isempty(body.Collisions)
        error('park:MissingVisualGeometry','Body %s has no visual geometry for whole-arm plane checks.',body.Name);
    end
    P=unique(P,'rows');
    if size(P,1)>20 && rank(P-mean(P,1))==3, f=convhull(P); P=P(unique(f),:); end
    mass=0; com=[0 0 0];
    if b>1, mass=body.Mass; com=body.CenterOfMass; end
    g(end+1)=struct('name',body.Name,'vertices',P,'mass',mass,'com',com,'isBase',b==1); %#ok<AGROW>
end
end

function key=poseKey(q)
key=sprintf('%.17g,',q);
end

function diagnostic=curvature(model,q,limits,tau)
n=numel(q); H=zeros(n);
for j=1:n
    a=q; b=q; a(j)=max(limits(j,1),q(j)-1e-4); b(j)=min(limits(j,2),q(j)+1e-4);
    H(:,j)=(gravityTorque(model,b)-gravityTorque(model,a)).'/(b(j)-a(j));
end
H=(H+H.')/2; eigenvalues=eig(H);
passive=max(abs(tau))<1e-3 && min(eigenvalues)>=-1e-4;
label="Active holding/brakes required; not certified for power-off parking.";
if passive, label="Near-zero torque, no negative local gravity curvature; brake safety remains unverified."; end
diagnostic=struct('potentialHessian',H,'eigenvalues',eigenvalues, ...
    'passiveEquilibrium',passive,'passesGravityCheck',min(eigenvalues)>=-1e-4,'label',label);
end
