function objective = ObjectiveFcn_Collision(x, dhparams_full, jointTypes, goalPoses, q_home)
    % Update the DH parameters using the current design variables.
    n = size(dhparams_full,1);
    dhparams_opt = dhparams_full;
    sum_design = 0;
    for i = 7:n
        % Extract current design variables for joint i.
        a_i = x{1, sprintf('a%d', i)};
        d_i = x{1, sprintf('d%d', i)};
        dhparams_opt(i,1) = a_i;  % update 'a'
        dhparams_opt(i,3) = d_i;  % update 'd'
        % Sum absolute values for design cost (or use squared terms if preferred)
        sum_design = sum_design + abs(a_i) + abs(d_i);
    end

    % Create the robot model using the updated DH parameters.
    robot = createRobotModel_Generalized(dhparams_opt, jointTypes, q_home);

% Define joint limits
jointLimits = [
    0, 10;     % Joint 2
    -pi, pi;   % Joint 3
    -pi, pi;   % Joint 4
    -1, 0;    % Joint 5
    -pi, pi;   % Joint 6
    -pi, pi;   % Joint 7
    -pi, pi;   % Joint 8
    -pi, pi;   % Joint 9
    -pi, pi;
];

    % Apply joint limits to the robot
    robot.getBody("body2").Joint.PositionLimits = [0, 10];
    robot.getBody("body6").Joint.PositionLimits = [-1, 0];

    % Initialize the constrained IK solver
    gik = generalizedInverseKinematics('RigidBodyTree', robot, ...
                                       'ConstraintInputs', {'pose', 'joint'});
    
    % Define joint bounds constraint
    jointBounds = constraintJointBounds(robot);
    jointBounds.Bounds = jointLimits;

    % Set up error accumulators
    numGoals = size(goalPoses,1);
    totalPoseError = 0;
    totalPenalty = 0;
    
    % Use q_home as the initial guess for the first goal
    q_guess = q_home;

    % Loop over each goal pose
    for i = 1:numGoals
        % Form the goal transformation
        pos_goal = goalPoses(i, 1:3);
        axang_goal = goalPoses(i, 4:7);
        T_goal = trvec2tform(pos_goal) * axang2tform(axang_goal);
        
        % Define a pose constraint for the end-effector
        poseTgt = constraintPoseTarget(robot.BodyNames{end});
        poseTgt.TargetTransform = T_goal;

        % Solve the constrained IK problem (now with three inputs)
        [q_sol, solInfo] = gik(q_guess, poseTgt, jointBounds);

        % If IK fails, add a large penalty
        if solInfo.Status ~= "success"
            totalPenalty = totalPenalty + 1000;
        end
        
        % Compute the achieved end-effector pose
        T_ee = getTransform(robot, q_sol, robot.BodyNames{end});
        
        % Compute the relative transformation error
        T_err = inv(T_goal) * T_ee;
        p_err = T_err(1:3,4);  % Translation error
        R_err = T_err(1:3,1:3); % Rotation error
        theta_err = acos(min(max((trace(R_err) - 1)/2, -1), 1));  % Angle difference
        
        % Compute unified pose error
        pose_error = sqrt(norm(p_err)^2 + theta_err^2);
        
        totalPoseError = totalPoseError + pose_error;
        
        % Update the initial guess for the next goal
        q_guess = q_sol;
    end

    % Define weights for objective components
    penaltyWeight = 1;   % IK failure penalty weight
    designWeight  = 1;   % Design parameter weight

    % Final objective: pose error + weighted design cost + penalty
    objective = totalPoseError + penaltyWeight * totalPenalty + designWeight * sum_design;

    fprintf('Objective: %.4f, Pose Error: %.4f, Penalty: %.4f, Design Sum: %.4f\n',...
        objective, totalPoseError, penaltyWeight*totalPenalty, sum_design);
end
