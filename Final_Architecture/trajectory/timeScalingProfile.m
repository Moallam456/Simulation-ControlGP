function profile = timeScalingProfile(lengthMeters,motion)
% TIMESCALINGPROFILE Convert physical TCP limits to normalized path motion.
required = {'tcpSpeed','tcpAcceleration','timeStep','profile'};
if ~all(isfield(motion,required)) || ~isscalar(lengthMeters) || ...
        ~isfinite(lengthMeters) || lengthMeters <= 0
    error('trajectory:InvalidMotion','Motion needs positive length, speed, acceleration, timeStep, and profile.');
end
v = motion.tcpSpeed/lengthMeters;
a = motion.tcpAcceleration/lengthMeters;
dt = motion.timeStep;
if any(~isfinite([v a dt])) || any([v a dt] <= 0)
    error('trajectory:InvalidMotion','Physical motion limits must be positive finite scalars.');
end
switch lower(string(motion.profile))
    case "scurve"
        if ~isfield(motion,'tcpJerk') || ~isscalar(motion.tcpJerk) || ...
                ~isfinite(motion.tcpJerk) || motion.tcpJerk <= 0
            error('trajectory:MissingJerk','S-curve requires positive tcpJerk.');
        end
        j = motion.tcpJerk/lengthMeters;
        if v < a^2/j
            tJ = sqrt(v/j);
            if 1 >= 2*v*tJ
                vPeak = v; tA = 0; tV = (1-2*v*tJ)/vPeak;
            else
                vPeak = (sqrt(j)/2)^(2/3);
                tJ = sqrt(vPeak/j); tA = 0; tV = 0;
            end
        else
            tJ = a/j;
            distanceForV = v*(tJ+v/a);
            if 1 >= distanceForV
                vPeak = v; tA = v/a-tJ; tV = (1-distanceForV)/v;
            elseif 1 >= 2*a^3/j^2
                vPeak = (-a^2/j+sqrt((a^2/j)^2+4*a))/2;
                tA = vPeak/a-tJ; tV = 0;
            else
                vPeak = (sqrt(j)/2)^(2/3);
                tJ = sqrt(vPeak/j); tA = 0; tV = 0;
            end
        end
        durations = [tJ tA tJ tV tJ tA tJ];
        jerks = [j 0 -j 0 -j 0 j];
        [t,s,sd,sdd,sddd] = evaluateConstantJerk(durations,jerks,dt);
        profile.accelerationContinuous = true;
    case "trapezoidal"
        vPeak = min(v,sqrt(a));
        tA = vPeak/a;
        tV = (1-vPeak^2/a)/vPeak;
        durations = [tA tV tA];
        T = sum(durations);
        t = makeTime(T,dt);
        s = zeros(size(t)); sd = s; sdd = s; sddd = NaN(size(t));
        for k = 1:numel(t)
            if t(k) < tA
                s(k)=0.5*a*t(k)^2; sd(k)=a*t(k); sdd(k)=a;
            elseif t(k) < tA+tV
                s(k)=0.5*vPeak*tA+vPeak*(t(k)-tA); sd(k)=vPeak;
            else
                h=t(k)-tA-tV;
                s(k)=1-0.5*vPeak*tA+vPeak*h-0.5*a*h^2;
                sd(k)=vPeak-a*h; sdd(k)=-a;
            end
        end
        profile.accelerationContinuous = false;
    otherwise
        error('trajectory:UnknownProfile','Use scurve or trapezoidal.');
end
if abs(s(end)-1)>1e-8 || any(diff(s)<-1e-10) || ...
        max(sd)>v+1e-8 || max(abs(sdd))>a+1e-8 || ...
        profile.accelerationContinuous && max(abs(sddd))>j+1e-8
    error('trajectory:InvalidTimeProfile', ...
        'Time profile does not cover the path within its motion limits.');
end
s(1)=0; s(end)=1; sd([1 end])=0;
if profile.accelerationContinuous, sdd([1 end])=0; end
profile.time=t(:);
profile.s=s(:);
profile.sd=sd(:);
profile.sdd=sdd(:);
profile.sddd=sddd(:);
profile.duration=t(end);
profile.pathLength=lengthMeters;
profile.tcpSpeed=lengthMeters*profile.sd;
profile.tcpAcceleration=lengthMeters*profile.sdd;
profile.tcpJerk=lengthMeters*profile.sddd;
profile.normalizedLimits=[v a];
if isfield(motion,'tcpJerk') && ~isempty(motion.tcpJerk)
    profile.normalizedLimits(3)=motion.tcpJerk/lengthMeters;
end
end

function [t,s,sd,sdd,sddd] = evaluateConstantJerk(durations,jerks,dt)
T=sum(durations);
t=makeTime(T,dt);
starts=[0 cumsum(durations(1:end-1))];
s0=zeros(size(durations)); v0=s0; a0=s0;
for k=2:numel(durations)
    h=durations(k-1); j=jerks(k-1);
    s0(k)=s0(k-1)+v0(k-1)*h+0.5*a0(k-1)*h^2+j*h^3/6;
    v0(k)=v0(k-1)+a0(k-1)*h+0.5*j*h^2;
    a0(k)=a0(k-1)+j*h;
end
s=zeros(size(t)); sd=s; sdd=s; sddd=s;
ends=cumsum(durations);
for i=1:numel(t)
    k=find(t(i)<=ends+1e-12,1,'first');
    if isempty(k), k=numel(durations); end
    h=t(i)-starts(k); j=jerks(k);
    s(i)=s0(k)+v0(k)*h+0.5*a0(k)*h^2+j*h^3/6;
    sd(i)=v0(k)+a0(k)*h+0.5*j*h^2;
    sdd(i)=a0(k)+j*h;
    sddd(i)=j;
end
end

function t = makeTime(T,dt)
t=(0:dt:T).';
if T-t(end)>1e-12, t(end+1)=T; else, t(end)=T; end
end
