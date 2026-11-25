function pathProgressPlotExactComparison(demo,chord,exact,a,b,geometry,folder)
%PATHPROGRESSPLOTEXACTCOMPARISON Render a chord/exact comparison from its audited data.
fig = figure('Visible','off','Position',[100 100 1280 1160],'Color','w');
cleanup = onCleanup(@() close(fig));
layout = tiledlayout(fig,4,2,'TileSpacing','compact','Padding','compact');
title(layout,replace(demo,"fullcircle","full circle")+" | chord versus exact circle");
blue = [0.10 0.36 0.64];
orange = [0.88 0.32 0.13];
origin = geometry.teachingPoints(:,1);
p = (chord.arcGeometryPath-origin)*1000;
circle = exact.segments(1).exactCircle;
e = (pathProgressCircleGeometry(circle,(0:1200)/1200)-origin)*1000;
teach = (geometry.teachingPoints-origin)*1000;
ax = nexttile(layout);
plot(ax,e(1,:),e(2,:),'-','Color',orange,'LineWidth',1.4);
hold(ax,'on');
plot(ax,p(1,:),p(2,:),'--','Color',blue,'LineWidth',1.2);
plot(ax,teach(1,:),teach(2,:),'ko','MarkerSize',5);
padding = 0.10*circle.radius*1000;
xLimits = [min(e(1,:))-padding, max(e(1,:))+padding];
yLimits = [min(e(2,:))-padding, max(e(2,:))+padding];
xlim(ax,xLimits);
ylim(ax,yLimits);
pbaspect(ax,[diff(xLimits),diff(yLimits),1]);
separation = 1e6*max(vecnorm(chord.pPath-exact.pPath,2,1));
decorate(ax,sprintf('Requested paths | peak separation %.2f µm',separation), ...
    'X from start [mm]','Y from start [mm]');
legend(ax,{'Exact circle','Chord reference','Teaching points'},'Location','eastoutside');

t = exact.tGlobal;
ax = nexttile(layout);
plot(ax,t,a.desiredSpeed,'-','Color',blue);
hold(ax,'on');
plot(ax,t,a.twist.ReconstructedSpeed_m_s,'--','Color',blue);
plot(ax,t,b.desiredSpeed,'-','Color',orange);
plot(ax,t,b.twist.ReconstructedSpeed_m_s,':','Color',orange,'LineWidth',1.4);
decorate(ax,'Desired and reconstructed TCP speed','Time [s]','Speed [m/s]');
legend(ax,{'Chord desired','Chord reconstructed','Exact desired','Exact reconstructed'}, ...
    'Location','best','NumColumns',2);
pair(nexttile(layout),t,abs(a.twist.TangentialError_m_s)*1000, ...
    abs(b.twist.TangentialError_m_s)*1000,blue,orange, ...
    'Tangential speed error magnitude','Error [mm/s]',true);
pair(nexttile(layout),t,a.twist.CrossPathError_m_s*1000,b.twist.CrossPathError_m_s*1000, ...
    blue,orange,'Perpendicular velocity error','Error [mm/s]',true);
pair(nexttile(layout),t,max(abs(chord.qDot),[],1),max(abs(exact.qDot),[],1), ...
    blue,orange,'Joint velocity envelope','Max |velocity| [rad/s]',false);
pair(nexttile(layout),t,max(abs(chord.qDDot),[],1),max(abs(exact.qDDot),[],1), ...
    blue,orange,'Joint acceleration envelope','Max |acceleration| [rad/s^2]',false);
ax = nexttile(layout);
pair(ax,t,max(abs(chord.qJerk),[],1),max(abs(exact.qJerk),[],1), ...
    blue,orange,'Sampled joint jerk and continuous peaks','Max |jerk| [rad/s^3]',true);
semilogy(ax,a.peaks.Time_s(2),a.peaks.AbsJerk_rad_s3(2),'o','Color',blue, ...
    'MarkerFaceColor',blue,'HandleVisibility','off');
semilogy(ax,b.peaks.Time_s(2),b.peaks.AbsJerk_rad_s3(2),'o','Color',orange, ...
    'MarkerFaceColor',orange,'HandleVisibility','off');
legend(ax,'Location','southeast');

ax = nexttile(layout);
dof = size(exact.qPath,1);
knotTrace(ax,a.knots,dof,blue,'-');
hold(ax,'on');
knotTrace(ax,b.knots,dof,orange,'-');
knotTrace(ax,a.phases,dof,blue,'x');
knotTrace(ax,b.phases,dof,orange,'x');
decorate(ax,'One-sided jerk change at knots and phases','Time [s]','Max |jump| [rad/s^3]');
legend(ax,{'Chord knots','Exact knots','Chord phases','Exact phases'},'Location','best','NumColumns',2);
exportgraphics(fig,fullfile(folder,demo+"_comparison.png"),'Resolution',150);
end

function pair(ax,t,chord,exact,blue,orange,heading,y,logScale)
if logScale
    chord(chord==0) = NaN;
    exact(exact==0) = NaN;
    semilogy(ax,t,chord,'-','Color',blue);
    hold(ax,'on');
    semilogy(ax,t,exact,'--','Color',orange);
else
    plot(ax,t,chord,'-','Color',blue);
    hold(ax,'on');
    plot(ax,t,exact,'--','Color',orange);
end
decorate(ax,heading,'Time [s]',y);
legend(ax,{'Chord reference','Exact reference'},'Location','best');
end

function knotTrace(ax,rows,dof,color,style)
amplitude = max(abs(reshape(rows.Jump_rad_s3,dof,[])),[],1);
amplitude(amplitude==0) = NaN;
semilogy(ax,rows.Time_s(1:dof:end),amplitude,style,'Color',color);
end

function decorate(ax,heading,x,y)
title(ax,heading);
xlabel(ax,x);
ylabel(ax,y);
grid(ax,'on');
end
