function pathProgressPlotEndpointSettling(output,summary,traces)
%PATHPROGRESSPLOTENDPOINTSETTLING Plot each endpoint and the deliberate timeout.
fig = figure('Visible','off','Color','w','Position',[100 100 1100 940]);
cleanup = onCleanup(@() close(fig));
layout = tiledlayout(fig,4,2,'TileSpacing','compact','Padding','compact');
title(layout,'Endpoint stops | 50 ms tick, 50 ms lag, 10 ms delay, offset start');
for k = 1:4
    if k<=3
        old = string(sprintf('path%d_preset3_stop0',k));
        id = string(sprintf('path%d_preset3_stop1',k));
        periodic = traces.(old);
    else
        id = "forced_timeout";
        periodic = [];
    end
    r = summary(summary.Scenario==id,:);
    gated = traces.(id);
    ax = nexttile(layout);
    draw(ax,periodic,gated,'PositionError_rad',r.PositionTolerance_rad,r.Outcome);
    title(ax,r.Case+" | "+r.Outcome);
    ylabel(ax,'Max joint error [rad]');
    ax = nexttile(layout);
    draw(ax,periodic,gated,'ActualSpeed_rad_s',r.VelocityTolerance_rad_s,r.Outcome);
    title(ax,'Actual joint velocity');
    ylabel(ax,'Max actual speed [rad/s]');
    if k==1
        legend(ax,{'Periodic trace','Periodic end','Gated trace','Gate tolerance'},'Location','northeast');
    end
end
exportgraphics(fig,fullfile(output,'endpoint_stop_comparison.png'),'Resolution',150);
end

function draw(ax,periodic,gated,field,tol,status)
hold(ax,'on');
if ~isempty(periodic)
    plot(ax,periodic.TimeFromEndpoint_s,periodic.(field),'--','Color',[.13 .40 .69]);
    plot(ax,0,periodic.(field)(end),'o','Color',[.13 .40 .69]);
end
plot(ax,gated.TimeFromEndpoint_s,gated.(field),'Color',[.77 .22 .13],'LineWidth',1.3);
yline(ax,tol,':','Color',[.23 .23 .23]);
xline(ax,0,':','Endpoint','HandleVisibility','off');
xline(ax,gated.TimeFromEndpoint_s(end),'--',status,'Color',[.77 .22 .13], ...
    'HandleVisibility','off');
xlim(ax,[-.15,max(.14,gated.TimeFromEndpoint_s(end)+.09)]);
xlabel(ax,'Wall time relative to planned endpoint [s]');
grid(ax,'on');
end
