function sys = admittance_1d_state_space(m, b)
% Continuous-time 1D admittance state-space model
%
% States: [position; velocity]
% Input:  Applied force [N]
% Outputs: Position [m], Velocity [m/s]

    arguments
        m (1,1) double {mustBePositive}
        b (1,1) double {mustBePositive}
    end

    A = [0,  1;
         0, -b/m];

    B = [0;
         1/m];

    C = eye(2);

    D = zeros(2,1);

    sys = ss(A, B, C, D);

    sys.StateName = {'Position', 'Velocity'};
    sys.InputName = {'Force'};
    sys.OutputName = {'Position', 'Velocity'};
end