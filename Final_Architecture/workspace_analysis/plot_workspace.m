function figures = plot_workspace(robot,points,metrics,options)

% PLOT_WORKSPACE Plot workspace point cloud and planar density maps.

figures.pointCloud = [];
figures.xyDensity = [];
figures.xzDensity = [];
figures.yzDensity = [];
figures.savedFiles = strings(0,1);

if ~options.showPlot && ~options.saveFigure
    return;
end

if options.saveFigure && ~isfolder(options.outputFolder)
    mkdir(options.outputFolder);
end

visibleState = 'on';

if ~options.showPlot
    visibleState = 'off';
end

figures.pointCloud = figure( ...
    'Name',sprintf('Reachable Workspace - %s',robot.id), ...
    'Visible',visibleState);

scatter3(points(:,1),points(:,2),points(:,3),4,'filled');
grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title(sprintf('Reachable Workspace: %s (%d Samples)', ...
    robot.id,options.numSamples),'Interpreter','none');
view(135,25);

figures.xyDensity = plotDensity(metrics.density.xy,'X (m)','Y (m)', ...
    sprintf('XY Density - %s',robot.id),visibleState);
figures.xzDensity = plotDensity(metrics.density.xz,'X (m)','Z (m)', ...
    sprintf('XZ Density - %s',robot.id),visibleState);
figures.yzDensity = plotDensity(metrics.density.yz,'Y (m)','Z (m)', ...
    sprintf('YZ Density - %s',robot.id),visibleState);

if options.saveFigure
    figures.savedFiles = saveFigures(figures,options.outputFolder,robot.id);
end

end

function fig = plotDensity(density,xLabelText,yLabelText,titleText,visibleState)

fig = figure('Name',titleText,'Visible',visibleState);

imagesc( ...
    density.edgesA, ...
    density.edgesB, ...
    density.counts.');

set(gca,'YDir','normal');
axis equal tight;
colorbar;
xlabel(xLabelText);
ylabel(yLabelText);
title(titleText,'Interpreter','none');

end

function savedFiles = saveFigures(figures,outputFolder,robotID)

items = {
    figures.pointCloud, "point_cloud"
    figures.xyDensity, "xy_density"
    figures.xzDensity, "xz_density"
    figures.yzDensity, "yz_density"};

savedFiles = strings(0,1);

for i = 1:size(items,1)
    fig = items{i,1};
    name = items{i,2};

    if isempty(fig) || ~isvalid(fig)
        continue;
    end

    filePath = fullfile(outputFolder,sprintf('%s_%s.png',robotID,name));
    exportgraphics(fig,filePath,'Resolution',150);
    savedFiles(end+1,1) = string(filePath); %#ok<AGROW>
end

end
