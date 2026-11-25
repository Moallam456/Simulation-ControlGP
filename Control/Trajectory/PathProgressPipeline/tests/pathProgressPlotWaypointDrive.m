function pathProgressPlotWaypointDrive(folder,S,traces)
%PATHPROGRESSPLOTWAYPOINTDRIVE Render the selected event traces and sweep.
blue = [.15 .36 .65]; red = [.80 .26 .15]; green = [.06 .52 .42];
fig = newFigure([1100 720]);
cleanup = onCleanup(@() close(fig));
layout = tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
title(layout,'Waypoint stops | 50 ms commands, offset start, ideal drive');
a = selected(S,traces,S.Group=="Stop comparison" & S.Initial=="Offset" & S.Period_ms==50 & ~S.Stops);
b = selected(S,traces,S.Group=="Stop comparison" & S.Initial=="Offset" & S.Period_ms==50 & S.Stops);
ax = nexttile(layout);
pair(ax,a.t,a.referenceTime,b.t,b.referenceTime,blue,red);
label(ax,'Moving reference and settling holds','Wall time [s]','Reference time [s]');
legend(ax,{'Periodic','Boundary-aware'},'Location','best');
ax = nexttile(layout);
pair(ax,a.t,a.tcpError*1000,b.t,b.tcpError*1000,blue,red);
label(ax,'Equal-reference-time tracking','Wall time [s]','TCP error [mm]');
for k = 1:2
    ax = nexttile(layout);
    pair(ax,a.t-a.boundaries(k),a.actualTcpSpeed*1000, ...
        b.t-b.boundaries(k),b.actualTcpSpeed*1000,blue,red);
    xline(ax,0,':','Boundary','HandleVisibility','off');
    if isfinite(b.releases(k))
        xline(ax,b.releases(k)-b.boundaries(k),'--','Gate release','Color',red,'HandleVisibility','off');
    end
    xlim(ax,[-.12,max(.3,b.releases(k)-b.boundaries(k)+.15)]);
    label(ax,sprintf('Waypoint %c',char('A'+k)),'Wall time relative to boundary [s]','Actual TCP speed [mm/s]');
end
exportgraphics(fig,fullfile(folder,'waypoint_behavior.png'),'Resolution',150);
clear cleanup

fig = newFigure([1100 660]);
cleanup = onCleanup(@() close(fig));
layout = tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');
title(layout,'Exact-length circle | 50 ms, Kp = 8/s, matched start');
mask = S.Group=="Matrix" & S.Case=="Exact-length circle" & S.Period_ms==50 & S.Gain_per_s==8;
a = selected(S,traces,mask & S.Tau_ms==0 & S.Delay_ms==0);
b = selected(S,traces,mask & S.Tau_ms==50 & S.Delay_ms==0);
c = selected(S,traces,mask & S.Tau_ms==50 & S.Delay_ms==10);
ax = nexttile(layout);
plot(ax,c.t,c.vd(3,:),'k--','LineWidth',1.3);
hold(ax,'on');
plot(ax,a.t,a.v(3,:),'Color',blue);
plot(ax,b.t,b.v(3,:),'Color',green);
plot(ax,c.t,c.v(3,:),'Color',red);
xlim(ax,[5.7 6.55]);
label(ax,'Desired and actual joint 3 velocity','Wall time [s]','Joint velocity [rad/s]');
legend(ax,{'Desired','Ideal','50 ms lag','50 ms lag + 10 ms delay'},'Location','southwest');
ax = nexttile(layout);
pair(ax,a.t,a.tcpError*1000,b.t,b.tcpError*1000,blue,green);
plot(ax,c.t,c.tcpError*1000,'Color',red);
label(ax,'Equal-time TCP tracking','Wall time [s]','TCP error [mm]');
legend(ax,{'Ideal','50 ms lag','50 ms lag + 10 ms delay'},'Location','northwest');
exportgraphics(fig,fullfile(folder,'lag_and_delay.png'),'Resolution',150);
clear cleanup

fig = newFigure([1200 510]);
cleanup = onCleanup(@() close(fig));
layout = tiledlayout(fig,1,3,'TileSpacing','compact','Padding','compact');
title(layout,'Matched-start sensitivity sweep');
for k = 1:3
    ax = nexttile(layout);
    if k==1
        grid = matrix(S,"Exact-length circle","RmsTcp_m",1000);
        heading = 'Circle RMS TCP error [mm]';
    elseif k==2
        grid = matrix(S,"Multi S-curve","RmsTcp_m",1000);
        heading = 'Multi RMS TCP error [mm]';
    else
        grid = matrix(S,"Multi S-curve","Settling_s",1);
        heading = 'Multi total settling [s]';
    end
    image = imagesc(ax,grid);
    image.AlphaData = isfinite(grid);
    ax.Color = [.92 .92 .92];
    [missingRow,missingCol] = find(isnan(grid));
    for j = 1:numel(missingRow)
        text(ax,missingCol(j),missingRow(j),'T','HorizontalAlignment','center', ...
            'Color',[.35 .12 .12],'FontWeight','bold');
    end
    set(ax,'XTick',1:6,'XTickLabel',{'0/0','0/10','10/0','10/10','50/0','50/10'}, ...
        'YTick',1:6,'YTickLabel',{'2/2','2/8','10/2','10/8','50/2','50/8'});
    colorbar(ax);
    title(ax,heading);
    if k==3, subtitle(ax,'T = timeout'); end
    xlabel(ax,'Lag / delay [ms]');
    if k==1, ylabel(ax,'Period [ms] / gain [1/s]'); end
end
exportgraphics(fig,fullfile(folder,'sensitivity.png'),'Resolution',150);
end

function values = matrix(S,name,field,scale)
values = nan(6,6);
i = 0;
for p = [2,10,50]
    for g = [2,8]
        i = i+1;
        j = 0;
        for tau = [0,10,50]
            for delay = [0,10]
                j = j+1;
                row = S(S.Group=="Matrix" & S.Case==name & S.Period_ms==p & ...
                    S.Gain_per_s==g & S.Tau_ms==tau & S.Delay_ms==delay,:);
                if row.Outcome=="Complete", values(i,j) = row.(field)*scale; end
            end
        end
    end
end
end

function trace = selected(S,traces,mask)
trace = traces.(S.Scenario(mask));
end

function fig = newFigure(size)
fig = figure('Visible','off','Color','w','Position',[100 100 size]);
end

function pair(ax,t,y,t2,y2,a,b)
plot(ax,t,y,'Color',a); hold(ax,'on'); plot(ax,t2,y2,'Color',b);
end

function label(ax,heading,x,y)
title(ax,heading); xlabel(ax,x); ylabel(ax,y); grid(ax,'on');
end
