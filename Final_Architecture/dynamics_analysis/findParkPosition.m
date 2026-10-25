function result = findParkPosition(robot,settings)
% FINDPARKPOSITION Compatibility alias; the sole engine is the optimizer below.
if nargin<2, settings=struct(); end
result=optimizeParkConfiguration(robot,settings);
end
