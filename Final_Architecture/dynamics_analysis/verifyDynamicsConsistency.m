function report = verifyDynamicsConsistency(robot,options)
% VERIFYDYNAMICSCONSISTENCY Reproducible numerical identities for a robot.
if nargin < 2, options = struct(); end
if ~isfield(options,'gravity'), options.gravity = [0 0 -9.81]; end
if ~isfield(options,'numSamples'), options.numSamples = 20; end
if ~isfield(options,'seed'), options.seed = 7; end
[model,validation] = prepareDynamicsModel(robot,options.gravity);
if ~isscalar(options.numSamples) || options.numSamples < 1 || ...
        options.numSamples ~= floor(options.numSamples)
    error('dynamics:InvalidSampleCount','numSamples must be a positive integer.');
end
prior = rng;
restore = onCleanup(@() rng(prior));
rng(options.seed,'twister');
limits = robot.params.joints.positionLimits;
dof = robot.structure.dof;
report.maxDecompositionResidual_Nm = 0;
report.maxGravityIdentityError_Nm = 0;
report.maxForwardInverseError_rad_s2 = 0;
report.maxMassMatrixAsymmetry_kg_m2 = 0;
report.minMassMatrixEigenvalue_kg_m2 = Inf;
report.maxZeroGravityStaticError_Nm = 0;
report.maxMassMatrixCondition = 0;
report.failureQ = [];
zeroModel = copy(model);
zeroModel.Gravity = [0 0 0];
for k = 1:options.numSamples
    q = limits(:,1).' + rand(1,dof).*(limits(:,2)-limits(:,1)).';
    qd = 0.5*(2*rand(1,dof)-1);
    qdd = 2*rand(1,dof)-1;
    M = massMatrix(model,q);
    g = gravityTorque(model,q);
    v = velocityProduct(model,q,qd);
    tau = inverseDynamics(model,q,qd,qdd);
    residual = tau - (M*qdd.').' - v - g;
    symmetry = norm(M-M.','fro');
    eigenvalue = min(eig((M+M.')/2));
    accelError = norm(forwardDynamics(model,q,qd,tau)-qdd,Inf);
    gravityError = norm(inverseDynamics(model,q,zeros(1,dof),zeros(1,dof))-g,Inf);
    zeroError = norm(inverseDynamics(zeroModel,q,zeros(1,dof),zeros(1,dof)),Inf);
    report.maxDecompositionResidual_Nm = max(report.maxDecompositionResidual_Nm,norm(residual,Inf));
    report.maxGravityIdentityError_Nm = max(report.maxGravityIdentityError_Nm,gravityError);
    report.maxForwardInverseError_rad_s2 = max(report.maxForwardInverseError_rad_s2,accelError);
    report.maxMassMatrixAsymmetry_kg_m2 = max(report.maxMassMatrixAsymmetry_kg_m2,symmetry);
    report.minMassMatrixEigenvalue_kg_m2 = min(report.minMassMatrixEigenvalue_kg_m2,eigenvalue);
    report.maxZeroGravityStaticError_Nm = max(report.maxZeroGravityStaticError_Nm,zeroError);
    report.maxMassMatrixCondition = max(report.maxMassMatrixCondition,cond(M));
    scale = max(1,norm(tau,Inf));
    if norm(residual,Inf) > 1e-9*scale || gravityError > 1e-9*scale || ...
            accelError > 1e-8 || symmetry > 1e-10*max(1,norm(M,'fro')) || ...
            eigenvalue <= 0 || zeroError > 1e-9*scale
        report.failureQ = [report.failureQ; q];
    end
end
report.numSamples = options.numSamples;
report.seed = options.seed;
report.dataQuality = validation.dataQuality;
report.pass = isempty(report.failureQ);
end
