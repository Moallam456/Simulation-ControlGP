function pathProgressPlotVelocityExecution(output,S,traces)
%PATHPROGRESSPLOTVELOCITYEXECUTION Plot the sweep and selected held traces.
colors = [.12 .36 .65;.80 .46 .10;.78 .19 .16];
labels = ["Feedforward","Feedback Kp=2","Feedback Kp=8"];
fig = figure('Visible','off','Color','w','Position',[100 100 1120 760]);
life = onCleanup(@() close(fig));
layout = tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
title(layout,'Motion RMS TCP tracking error versus command period');
gains = [0 2 8];
for name = unique(S.Case,'stable').'
    ax = nexttile(layout);
    hold(ax,'on');
    for k = 1:3
        for initial = ["Matched","Offset"]
            rows = S(S.Case==name & S.Initial==initial & S.Kp_per_s==gains(k),:);
            style = '-o';
            if initial=="Offset", style = '--s'; end
            plot(ax,rows.Period_ms,rows.RmsTcp_m*1000,style,'Color',colors(k,:), ...
                'LineWidth',1.2,'DisplayName',labels(k)+" / "+initial);
        end
    end
    set(ax,'XScale','log','YScale','log','XTick',[1 2 5 10 20 50]);
    decorate(ax,replace(name,"_"," "),'Command period [ms]','RMS TCP error [mm]');
    if name==S.Case(1), legend(ax,'Location','northwest','FontSize',8); end
end
exportgraphics(fig,fullfile(output,'period_sweep.png'),'Resolution',150);
clear life

a = traces.fullcircle_exact_length_kp0;
b = traces.fullcircle_exact_length_kp8;
fig = figure('Visible','off','Color','w','Position',[100 100 1120 750]);
life = onCleanup(@() close(fig));
layout = tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
title(layout,'Full circle | 50 ms commands, offset start');
ax = nexttile(layout);
speedPlot(ax,a,b,colors);
xline(ax,a.duration,':','Completion','HandleVisibility','off');
decorate(ax,'Reference and executed TCP speed','Time [s]','Speed [m/s]');
ax = nexttile(layout);
plotPair(ax,a.time,rad2deg(a.jointError(3,:)),rad2deg(b.jointError(3,:)),colors);
decorate(ax,'Joint 3 tracking error','Time [s]','Signed error [deg]');
legend(ax,{'Feedforward','Feedback Kp=8'},'Location','best');
ax = nexttile(layout);
stairs(ax,a.ticks,a.command(3,:),'Color',colors(1,:));
hold(ax,'on');
stairs(ax,b.ticks,b.command(3,:),'Color',colors(3,:));
xlim(ax,[5.4 6.4]);
decorate(ax,'Held joint 3 velocity','Time [s]','Command [rad/s]');
ax = nexttile(layout);
semilogy(ax,a.time,max(a.positionError*1000,eps),'Color',colors(1,:));
hold(ax,'on');
semilogy(ax,b.time,max(b.positionError*1000,eps),'Color',colors(3,:));
xline(ax,a.duration,':','Completion');
decorate(ax,'Offset recovery and post-trajectory observation','Time [s]','TCP tracking error [mm]');
exportgraphics(fig,fullfile(output,'held_execution.png'),'Resolution',150);
clear life

a = traces.multi_scurve_kp0;
b = traces.multi_scurve_kp8;
fig = figure('Visible','off','Color','w','Position',[100 100 1080 675]);
life = onCleanup(@() close(fig));
layout = tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');
title(layout,'Multi-segment S-curve | 50 ms commands, offset start');
ax = nexttile(layout);
speedPlot(ax,a,b,colors);
markers(ax,a.waypoints);
decorate(ax,'TCP speed at waypoint stops','Time [s]','Speed [m/s]');
ax = nexttile(layout);
plotPair(ax,a.time,a.positionError*1000,b.positionError*1000,colors);
markers(ax,a.waypoints);
decorate(ax,'Tracking error at equal times','Time [s]','TCP tracking error [mm]');
exportgraphics(fig,fullfile(output,'waypoint_stops.png'),'Resolution',150);
end

function speedPlot(ax,a,b,colors)
plot(ax,a.time,a.desiredSpeed,'Color',[.2 .2 .2],'LineWidth',1.4);
hold(ax,'on');
plotPair(ax,a.time,a.executedSpeed,b.executedSpeed,colors);
legend(ax,{'Reference FK','Feedforward','Feedback Kp=8'},'Location','south');
end

function plotPair(ax,t,a,b,colors)
plot(ax,t,a,'Color',colors(1,:));
hold(ax,'on');
plot(ax,t,b,'Color',colors(3,:));
end

function markers(ax,times)
for k = 1:numel(times)
    xline(ax,times(k),':',sprintf('Waypoint %c','A'+k),'HandleVisibility','off');
end
end

function decorate(ax,heading,x,y)
title(ax,heading);
xlabel(ax,x);
ylabel(ax,y);
grid(ax,'on');
end
