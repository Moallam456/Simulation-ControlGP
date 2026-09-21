function chain = kinematicChain(params,structure)

% KINEMATICCHAIN Fixed joint placements and visual segments for initial_trial.
%
% Home frames describe the joint locations and orientations at q = 0.
% Each fixed transform places a joint relative to its parent; joint motion
% is applied separately about the axis in robotStructure.

validateParams(params,structure);

g = params.geometry;

chain.convention = "fixed_transform_chain";
[chain.homeFrames, chain.fixedTransforms] = homeTransforms(params);
chain.visualSegments = visualSegments(params);
chain.baseSegment = chain.visualSegments{1};
chain.bodySegments = cell(1,structure.dof);
for i = 1:structure.dof-1
    chain.bodySegments{i} = chain.visualSegments{i+1};
end
chain.bodySegments{end} = [0 0 0];
chain.toolSegment = [
    0 0 0
    0 0 g.l10];

end

function validateParams(params,structure)

required = ["l1","l2","l3","l4","l5","l6","l7","l8","l9","l10"];

for i = 1:numel(required)
    value = params.geometry.(required(i));

    if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value)
        error('initial_trial:InvalidGeometry', ...
            'params.geometry.%s must be a finite scalar in meters.', ...
            required(i));
    end
end

if size(params.joints.positionLimits,1) ~= structure.dof || ...
   size(params.joints.positionLimits,2) ~= 2
    error('initial_trial:InvalidJointLimits', ...
        'positionLimits must be a %d-by-2 matrix.', structure.dof);
end

end

function [homeFrames,fixedTransforms] = homeTransforms(params)

g = params.geometry;

p0 = [0; 0; 0];
p1 = p0 + [0; 0; g.l1];
p2 = p1 + [g.l3; 0; g.l2];
p3 = p2 + [0; 0; g.l4];
p4 = p3 + [g.l7; g.l5; g.l6];
p5 = p4 + [g.l8; 0; 0];
p6 = p5 + [g.l9; 0; 0];

R0 = eye(3);
R1 = eye(3);
R2 = eye(3);
R3 = eye(3);
R4 = frameFromAxes([0;0;1],[1;0;0]);
R5 = R4;
R6 = R4;

homeFrames = {
    makeTransform(R0,p0)
    makeTransform(R1,p1)
    makeTransform(R2,p2)
    makeTransform(R3,p3)
    makeTransform(R4,p4)
    makeTransform(R5,p5)
    makeTransform(R6,p6)};

fixedTransforms = cell(1,6);

for i = 1:6
    fixedTransforms{i} = homeFrames{i} \ homeFrames{i+1};
end

end

function segments = visualSegments(params)

g = params.geometry;

segments = cell(1,6);
segments{1} = [0 0 0; 0 0 g.l1];
segments{2} = [0 0 0; 0 0 g.l2; g.l3 0 g.l2];
segments{3} = [0 0 0; 0 0 g.l4];
segments{4} = [0 0 0; 0 g.l5 0; 0 g.l5 g.l6; g.l7 g.l5 g.l6];
segments{5} = [0 0 0; 0 0 g.l8];
segments{6} = [0 0 0; 0 0 g.l9];

end

function R = frameFromAxes(xAxis,zAxis)

xAxis = xAxis/norm(xAxis);
zAxis = zAxis/norm(zAxis);
yAxis = cross(zAxis,xAxis);
yAxis = yAxis/norm(yAxis);
zAxis = cross(xAxis,yAxis);
zAxis = zAxis/norm(zAxis);

R = [xAxis yAxis zAxis];

end

function T = makeTransform(R,p)

T = eye(4);
T(1:3,1:3) = R;
T(1:3,4) = p;

end
