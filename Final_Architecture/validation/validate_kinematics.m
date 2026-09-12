function validate_kinematics()

robot = robot_params();

clc;

fprintf('\n=====================================\n');
fprintf('KINEMATICS VALIDATION\n');
fprintf('=====================================\n\n');

numTests = 100;

positionTolerance = 1e-4;      % meters
orientationTolerance = 1e-4;   % radians

passed = 0;

for k = 1:numTests

    % Generate random configuration

    q = random_configuration(robot);

    % FK

    T_target = forward_kinematics(q, robot.DH);

    % IK

    q_ik = inverse_kinematics(T_target, robot);

    % FK again

    T_check = forward_kinematics(q_ik, robot.DH);

    % Compare

    posError = norm( ...
        T_target(1:3,4) - T_check(1:3,4));

    R_error = T_target(1:3,1:3)' * T_check(1:3,1:3);

    angError = acos( ...
        max(min((trace(R_error)-1)/2,1),-1));

    if posError < positionTolerance && ...
       angError < orientationTolerance

        passed = passed + 1;

    else

        fprintf('\nTest %d FAILED\n',k);
        fprintf('Position Error : %.8f m\n',posError);
        fprintf('Orientation Error : %.8f rad\n',angError);

    end

end

fprintf('\n=====================================\n');
fprintf('PASSED: %d / %d\n',passed,numTests);
fprintf('SUCCESS RATE: %.2f %%\n', ...
        100*passed/numTests);
fprintf('=====================================\n');

end