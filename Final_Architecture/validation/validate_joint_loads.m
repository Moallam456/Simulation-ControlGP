function report = validate_joint_loads(runUI)
% VALIDATE_JOINT_LOADS Independent equilibrium/inertial checks and UI smoke test.
if nargin < 1, runUI = false; end
root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));

% Hand-solvable rotating offset mass, with an inclined axis and translated
% joint. The heavy fixed pedestal must not appear in the Joint-1 reaction.
m = rigidBodyTree('DataFormat','row'); m.Gravity = [0 0 -9.81];
pedestal = rigidBody('pedestal'); pedestal.Mass = 100;
T = trvec2tform([.15 -.2 .3])*axang2tform([1 0 0 .5]);
setFixedTransform(pedestal.Joint,T); addBody(m,pedestal,m.BaseName);
b = rigidBody('arm'); b.Joint = rigidBodyJoint('j1','revolute');
b.Joint.JointAxis = [0 0 1]; b.Mass = 2; b.CenterOfMass = [.4 0 0];
b.Inertia = [.02 .03+2*.4^2 .04+2*.4^2 0 0 0]; addBody(m,b,'pedestal');
q = .37; qd = 1.2; qdd = -.7;
out = joint1ReactionLoads(m,q,qd,qdd);
R = T(1:3,1:3); axis = R*[0;0;1];
r = R*[.4*cos(q);.4*sin(q);0];
acceleration = cross(axis*qdd,r)+cross(axis*qd,cross(axis*qd,r));
expectedForce = 2*(acceleration-m.Gravity.');
expectedMoment = cross(r,expectedForce)+.04*qdd*axis;
report.analyticForceError_N = norm(out.forceBase_N.'-expectedForce);
report.analyticMomentError_Nm = norm(out.momentAtJointBase_Nm.'-expectedMoment);
assert(report.analyticForceError_N<1e-10 && report.analyticMomentError_Nm<1e-10);
assert(abs(out.supportedMass_kg-2)<1e-12);
assert(norm(out.jointOriginBase_m-T(1:3,4).')<1e-12);
assert(abs(out.torsionSigned_Nm-inverseDynamics(m,q,qd,qdd))<1e-10);
assert(norm(out.forceJoint_N-out.forceBase_N*R)<1e-12);
assert(m.Bodies{1}.Mass==100 && m.NumBodies==2);

% Known horizontal cantilever: vertical J1 has zero gravity axis torque,
% but nonzero bending. Changing q around the vertical preserves its magnitude.
setFixedTransform(m.Bodies{1}.Joint,eye(4));
out = joint1ReactionLoads(m,[0;.8;2]);
assert(max(abs(out.axial_N-19.62))<1e-10);
assert(max(out.radial_N)<1e-10 && max(out.torsion_Nm)<1e-10);
assert(max(abs(out.bending_Nm-7.848))<1e-10);

ids = ["initial_trial" "robot_ver1" "robot_ver2"];
stream = RandStream('mt19937ar','Seed',71);
report.robots = struct([]);
for index = 1:numel(ids)
    robot = loadRobot(ids(index));
    [model,~] = prepareDynamicsModel(robot,[0 0 -9.81]);
    beforeMass = cellfun(@(body) body.Mass,model.Bodies);
    beforeInertia = cellfun(@(body) body.Inertia,model.Bodies,'UniformOutput',false);
    n = robot.structure.dof;
    states = [robot.params.joints.homePosition; rand(stream,4,n)-.5];
    speeds = .5*(rand(stream,5,n)-.5);
    accels = .6*(rand(stream,5,n)-.5);
    stationary = joint1ReactionLoads(model,states);
    dynamic = joint1ReactionLoads(model,states,speeds,accels);
    torqueError = 0; forceError = 0; momentError = 0;
    accelerationError = 0; angularMomentumError = 0;
    for k = 1:size(states,1)
        original = inverseDynamics(model,states(k,:),speeds(k,:),accels(k,:));
        torqueError = max(torqueError,max(abs(dynamic.physicalJointTorques_Nm(k,:)-original)));
        torqueError = max(torqueError,abs(dynamic.torsionSigned_Nm(k)-original(1)));
        F = zeros(1,3); M = zeros(1,3);
        for name = stationary.supportedBodyNames
            body = getBody(model,char(name));
            Tb = getTransform(model,states(k,:),char(name));
            center = Tb*[body.CenterOfMass.';1];
            support = -body.Mass*model.Gravity;
            F = F+support;
            M = M+cross(center(1:3).'-stationary.jointOriginBase_m(k,:),support);
        end
        forceError = max(forceError,norm(F-stationary.forceBase_N(k,:)));
        momentError = max(momentError,norm(M-stationary.momentAtJointBase_Nm(k,:)));
        % Independent F = sum(m*a_COM)-sum(m*g), using centered FK differences.
        errors = zeros(1,2); momentErrors = zeros(1,2);
        for hIndex = 1:2
            h = .002/2^(hIndex-1);
            qm = states(k,:)-h*speeds(k,:)+.5*h*h*accels(k,:);
            qp = states(k,:)+h*speeds(k,:)+.5*h*h*accels(k,:);
            accelerationForce = (massPosition(model,qp,stationary.supportedBodyNames) ...
                -2*massPosition(model,states(k,:),stationary.supportedBodyNames) ...
                +massPosition(model,qm,stationary.supportedBodyNames))/h^2;
            errors(hIndex) = norm(accelerationForce+F-dynamic.forceBase_N(k,:));
            origin = stationary.jointOriginBase_m(k,:);
            Hm = angularMomentum(model,qm,speeds(k,:)-h*accels(k,:),stationary.supportedBodyNames,origin);
            Hp = angularMomentum(model,qp,speeds(k,:)+h*accels(k,:),stationary.supportedBodyNames,origin);
            momentErrors(hIndex) = norm((Hp-Hm)/(2*h)+M-dynamic.momentAtJointBase_Nm(k,:));
        end
        assert(errors(2)<max(1e-6,errors(1)));
        assert(momentErrors(2)<max(1e-6,momentErrors(1)));
        accelerationError = max(accelerationError,errors(2));
        angularMomentumError = max(angularMomentumError,momentErrors(2));
    end
    assert(torqueError<1e-8 && forceError<1e-8 && momentError<1e-8 && ...
        accelerationError<1e-5 && angularMomentumError<1e-5);
    assert(isequal(beforeMass,cellfun(@(body) body.Mass,model.Bodies)));
    assert(isequal(beforeInertia,cellfun(@(body) body.Inertia,model.Bodies,'UniformOutput',false)));
    report.robots(index).id = ids(index);
    report.robots(index).supportedMass_kg = stationary.supportedMass_kg;
    report.robots(index).torqueError_Nm = torqueError;
    report.robots(index).staticForceError_N = forceError;
    report.robots(index).staticMomentError_Nm = momentError;
    report.robots(index).dynamicForceError_N = accelerationError;
    report.robots(index).dynamicMomentError_Nm = angularMomentumError;
end

report.uiTested = logical(runUI);
if runUI
    visibility = get(groot,'DefaultFigureVisible');
    cleanupVisibility = onCleanup(@() set(groot,'DefaultFigureVisible',visibility));
    set(groot,'DefaultFigureVisible','off');
    session = force_analysis();
    cleanupSession = onCleanup(@() session.close());
    session.calculate();
    state = session.getState();
    assert(~isempty(state.result));
    slider = findobj(session.figure,'Style','slider');
    slider(1).Value = slider(1).Value+.01;
    state = session.getState();
    assert(isempty(state.result));
    session.search(24);
    state = session.getState();
    assert(~isempty(state.result) && state.collision.hasRequiredClearance);
    allCollision = analyzeCollisions(session.robot,state.search.loads.q, ...
        struct('minimumClearance',state.search.minimumClearance_m));
    acceptable = allCollision.hasRequiredClearance;
    metricSelect = findobj(session.figure,'Tag','StaticLoadMetric');
    assert(isscalar(metricSelect) && numel(metricSelect.String)==4);
    sampledQ = state.search.loads.q;
    fields = {'bending_Nm','axial_N','radial_N','torsion_Nm'};
    for metric = 1:4
        metricSelect.Value = metric;
        feval(metricSelect.Callback,metricSelect,[]);
        state = session.getState();
        chosen = state.search.selectedIndex;
        values = state.search.loads.(fields{metric});
        assert(strcmp(state.search.selectedMetric,fields{metric}));
        assert(state.search.maximumIndices(metric)==chosen && acceptable(chosen));
        assert(abs(values(chosen)-max(values(acceptable)))<=state.search.tieTolerance);
        assert(isequal(state.search.loads.q,sampledQ));
        assert(isequal(state.q,sampledQ(chosen,:)));
        assert(abs(state.result.(fields{metric})-values(chosen))<1e-9);
        assert(state.search.constantAcrossSamples== ...
            (max(values)-min(values)<=state.search.tieTolerance));
        if state.search.constantAcrossSamples
            assert(chosen==find(acceptable,1));
            status = findobj(session.figure,'Tag','StaticLoadStatus');
            assert(contains(status.String,'representative tied peak'));
        end
    end
    assert(isempty(findobj(session.figure,'String','Show worst found')));
    assert(isscalar(findobj(session.figure,'String','Find peak configuration')));
    if session.robot.id=="robot_ver2"
        staticState = session.getState();
        timerCount = numel(timerfindall('Name','Joint1ForcePlayback'));
        session.analyzeTrajectory("JOINT_SWEEP_180");
        viewer = session.getTrajectoryViewer();
        viewer.pause();
        state = session.getState();
        assert(~isempty(state.motion) && state.motion.validation.pass);
        assert(all(isfinite(state.motion.loads.forceBase_N),'all'));
        session.showTrajectorySample(20);
        state = session.getState();
        assert(norm(state.motionView.q-state.motion.loads.q(20,:))<1e-12);
        assert(norm(state.motionView.forceBase_N-state.motion.loads.forceBase_N(20,:))<1e-10);
        assert(isequal(state.q,staticState.q) && isequal(state.result,staticState.result));
        fields = {'bending_Nm','axial_N','radial_N','torsion_Nm'};
        for metric = 1:4
            viewer.showPeak(metric);
            current = viewer.getState();
            [~,expected] = max(state.motion.loads.(fields{metric}));
            assert(current.index==expected && ~current.playing);
            assert(all(abs(current.cursorTimes-state.motion.trajectory.time(expected))<1e-12));
        end
        scrubber = findobj(viewer.figure,'Style','slider');
        scrubber.Value = 37;
        feval(scrubber.Callback,scrubber,[]);
        current = viewer.getState();
        assert(current.index==37 && ~current.playing);
        speedMenu = findobj(viewer.figure,'Style','popupmenu');
        speedMenu.Value = 3;
        feval(speedMenu.Callback,speedMenu,[]);
        current = viewer.getState();
        assert(current.rate==2);
        speedMenu.Value = 2;
        feval(speedMenu.Callback,speedMenu,[]);
        viewer.restart();
        pause(1.0); drawnow;
        viewer.pause();
        current = viewer.getState();
        assert(current.index>1 && ~current.playing);
        pausedIndex = current.index;
        pause(.15); drawnow;
        current = viewer.getState();
        assert(current.index==pausedIndex);
        viewer.seek(numel(state.motion.trajectory.time));
        viewer.play();
        viewer.pause();
        current = viewer.getState();
        assert(current.index==1);
        report.trajectoryPeaks = [max(state.motion.loads.axial_N) max(state.motion.loads.radial_N) ...
            max(state.motion.loads.bending_Nm) max(state.motion.loads.torsion_Nm)];
        report.trajectoryClearance_m = min(state.motion.collision.minimumClearance);
        selected = session.robot.dynamicsCases(string({session.robot.dynamicsCases.id})=="JOINT_SWEEP_180");
        selected.motion.timeStep = selected.motion.timeStep/2;
        [fine,fineValidation] = generateTrajectory(session.robot,selected);
        assert(fineValidation.pass);
        [dynamicModel,~] = prepareDynamicsModel(session.robot,[0 0 -9.81]);
        fineLoads = joint1ReactionLoads(dynamicModel,fine.q,fine.qd,fine.qdd);
        finePeaks = [max(fineLoads.axial_N) max(fineLoads.radial_N) ...
            max(fineLoads.bending_Nm) max(fineLoads.torsion_Nm)];
        report.trajectoryPeakRefinementRelativeChange = ...
            abs(finePeaks-report.trajectoryPeaks)./max(1,abs(finePeaks));
        assert(all(report.trajectoryPeakRefinementRelativeChange<.02));
        % Direct figure deletion must clean up playback, not just CloseRequest.
        delete(viewer.figure);
        assert(numel(timerfindall('Name','Joint1ForcePlayback'))==timerCount);
        state = session.getState();
        assert(isequal(state.q,staticState.q) && isequal(state.result,staticState.result));
    end
end
report.pass = true;
disp(report);
end

function H = angularMomentum(model,q,qd,names,origin)
H = zeros(1,3);
for name = names
    body = getBody(model,char(name));
    T = getTransform(model,q,char(name)); R = T(1:3,1:3);
    J = geometricJacobian(model,q,char(name));
    omega = J(1:3,:)*qd.';
    offset = R*body.CenterOfMass.';
    velocity = J(4:6,:)*qd.'+cross(omega,offset);
    p = T(1:3,4)+offset-origin.';
    v = body.Inertia;
    Iorigin = [v(1) v(6) v(5);v(6) v(2) v(4);v(5) v(4) v(3)];
    c = body.CenterOfMass.';
    Icom = Iorigin-body.Mass*((c.'*c)*eye(3)-c*c.');
    H = H+(R*Icom*R.'*omega+cross(p,body.Mass*velocity)).';
end
end

function p = massPosition(model,q,names)
p = zeros(1,3);
for name = names
    body = getBody(model,char(name));
    T = getTransform(model,q,char(name));
    center = T*[body.CenterOfMass.';1];
    p = p+body.Mass*center(1:3).';
end
end
