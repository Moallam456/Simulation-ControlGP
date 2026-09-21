function figures = plotActuatorSizingEnvelope(suite)
% PLOTACTUATORSIZINGENVELOPE Plot paired operating points and case metrics.
a=suite.aggregate;
dof=numel(a.joints);
figures.envelope=figure('Name','Observed joint torque-speed envelope');
layout=tiledlayout(figures.envelope,ceil(dof/2),2,'TileSpacing','compact');
for j=1:dof
    ax=nexttile(layout); hold(ax,'on'); grid(ax,'on');
    data=a.torqueSpeed(j);
    ids=unique(data.caseID,'stable');
    for k=1:numel(ids)
        mask=data.caseID==ids(k);
        scatter(ax,data.speed_rad_s(mask),data.torque_Nm(mask),8, ...
            'filled','DisplayName',ids(k));
    end
    title(ax,a.joints(j).joint);
    xlabel(ax,'Speed (rad/s)'); ylabel(ax,'Torque (N m)');
    if j==1 && ~isempty(ids), legend(ax,'Location','best'); end
end
figures.comparison=[];
if ~isempty(a.caseComparison)
    figures.comparison=figure('Name','Load-case comparison');
    values=a.caseComparison;
    tiled=tiledlayout(figures.comparison,2,2,'TileSpacing','compact');
    columns={'PeakAbsTorque_Nm','RMSTorque_Nm', ...
        'PeakAbsSpeed_rad_s','PeakAbsPower_W'};
    labels={'Peak |torque| (N m)','Case RMS torque (N m)', ...
        'Peak |speed| (rad/s)','Peak |power| (W)'};
    for k=1:4
        ax=nexttile(tiled);
        ids=unique(values.CaseID,'stable');
        joints=unique(values.Joint,'stable');
        matrix=NaN(numel(ids),numel(joints));
        for row=1:height(values)
            matrix(ids==values.CaseID(row),joints==values.Joint(row))= ...
                values.(columns{k})(row);
        end
        imagesc(ax,matrix); colorbar(ax);
        set(ax,'XTick',1:numel(joints),'XTickLabel',joints, ...
            'YTick',1:numel(ids),'YTickLabel',ids);
        title(ax,labels{k});
    end
end
end
