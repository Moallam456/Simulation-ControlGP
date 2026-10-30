
function sys = admittance_1d_discrete_state_space(m, b, Ts)
% Backward Euler discrete-time admittance model
%
% States: [position; velocity]
% Input:  Force [N]
% Outputs: Position [m], Velocity [m/s]

    arguments
        m  (1,1) double {mustBePositive}
        b  (1,1) double {mustBePositive}
        Ts (1,1) double {mustBePositive}
    end

    %% Backward Euler coefficients
    alpha = m / (m + b*Ts);
    beta  = Ts / (m + b*Ts);

    %% Discrete state-space matrices
    Ad = [1, Ts*alpha;
          0, alpha];

    Bd = [Ts*beta;
          beta];

    Cd = eye(2);
    Dd = zeros(2,1);

    %% Discrete state-space system
    sys = ss(Ad, Bd, Cd, Dd, Ts);

    sys.StateName = {'Position', 'Velocity'};
    sys.InputName = {'Force'};
    sys.OutputName = {'Position', 'Velocity'};

end
