function objective = ObjectiveFcn_Collision(x, dhparams_full, jointTypes, goalPoses, q_home)
    % Update DH parameters using optimized variables.
    % (Assuming only joint 6’s 'a' and joints 7–11's 'd' are optimized.)
    dhparams_opt = dhparams_full;
    dhparams_opt(6,1)  = x.a6;
    dhparams_opt(7,3)  = x.d7;
    dhparams_opt(8,3)  = x.d8;
    dhparams_opt(9,3)  = x.d9;
    dhparams_opt(10,3) = x.d10;
    dhparams_opt(11,3) = x.d11;
    
    % Accumulate design cost:
    sum_design = abs(x.a6) + abs(x.d7) + abs(x.d8) + abs(x.d9) + abs(x.d10) + abs(x.d11);
    
    % Create the robot model using updated DH parameters.
    robot = createRobotCollisionModel(dhparams_opt, jointTypes, q_home);
    %disp(dhparams_opt)
    
    % Set joint limits
    jointLimits = [
        0, 10;     % Joint 2
        -pi, pi;   % Joint 3
        -pi, pi;   % Joint 4
        -1, 0;     % Joint 5
        -pi, pi;   % Joint 6
        -pi, pi;   % Joint 7
        -pi, pi;   % Joint 8
        -pi, pi;   % Joint 9
        -pi, pi;
    ];
    robot.getBody("body2").Joint.PositionLimits = [0, 10];
    robot.getBody("body6").Joint.PositionLimits = [-1, 0];
    robot.getBody("body6").Joint.HomePosition = -0.5;
    q_home = homeConfiguration(robot);

    % Define obstacle environment
    collisionCylinders = {
    collisionCylinder(0.457, 8.0)...  % Main Pipe
    collisionCylinder(0.044, 0.3)...  % Front BLM
    collisionCylinder(0.044, 0.3)...   % Back BLM
    collisionCylinder(0.350, 1.5)...  % First Obstacle
    collisionCylinder(0.350, 1.5)...   % Second Obstacle
    };

    cylinderPoses = [
    -0.958, 4.260, -1.707, 1, 0, 0, pi/2; 
    -0.458, 2.015, -1.627, 1, 0, 0, pi/2;
    -1.456, 6.921, -1.627, 1, 0, 0, pi/2; 
    -0.952, 5.400, -0.6, 1, 0, 0, 0;
    -0.952, 6.900, -0.6, 1, 0, 0, 0
    ];

    for i = 1:length(collisionCylinders)
        P = trvec2tform(cylinderPoses(i, 1:3));
        R = axang2tform(cylinderPoses(i, 4:7));
        collisionCylinders{i}.Pose = P * R;
    end
    
    % Initialize the constrained IK solver.
    gik = generalizedInverseKinematics('RigidBodyTree', robot, ...
                                       'ConstraintInputs', {'pose', 'joint'});
    jointBounds = constraintJointBounds(robot);
    jointBounds.Bounds = jointLimits;

    planner = manipulatorRRT(robot,collisionCylinders);
    planner.SkippedSelfCollisions = 'parent';
    planner.EnableConnectHeuristic = false;
    planner.MaxConnectionDistance = 0.1;
    planner.ValidationDistance = 0.01;
    planner.MaxIterations = 10000;
    rng(0)
 
    % Initialize accumulators for pose error and penalties.
    numGoals = size(goalPoses, 1);
    totalPoseError = 0;
    totalPenalty = 0;
    
    % Use the home configuration as the starting guess.
    q_guess = q_home;

    % Check for self collision in home configuration
    [isSelfColliding, selfSeparationDist, selfWitnessPts] = checkCollision(robot,q_guess, collisionCylinders, 'SkippedSelfCollisions','parent');
    bodynames = [robot.BodyNames robot.Base.Name];
    %collTable = array2table(selfSeparationDist,VariableNames=bodynames,RowNames=bodynames)

    %show(robot,q_guess, "Collisions","on");
   
    % % Wait for user input
    % disp('Press Enter to move to the first pose...');
    % pause;
    % 
    % Define a penalty values for collision or planning failure.
    collisionPenaltyValue = 1000; 
    IKPenaltyValue = 500.5;
    
    % Loop over each goal pose.
    for i = 1:numGoals
        % Form the target transformation.
        pos_goal = goalPoses(i, 1:3);
        axang_goal = goalPoses(i, 4:7);
        T_goal = trvec2tform(pos_goal) * axang2tform(axang_goal);
        
        % Define the pose constraint for the end-effector.
        poseTgt = constraintPoseTarget(robot.BodyNames{end});
        poseTgt.TargetTransform = T_goal;
        
        % Solve the constrained IK problem using the current guess.
        [q_sol, solInfo] = gik(q_guess, poseTgt, jointBounds);
        
        % If IK fails, add a penalty.
        if solInfo.Status ~= "success"
            totalPenalty = totalPenalty + IKPenaltyValue;
        end
        
        try
            rrtPath = planner.plan(q_guess, q_sol);
        catch ME
            warning('RRT planning failed: %s', ME.identifier, '%s', ME.message);
            rrtPath = [];
        end
        
        if isempty(rrtPath)
            totalPenalty = totalPenalty + collisionPenaltyValue;
        else
            % Optionally, check each configuration along the path for collisions.
            for j = 1:size(rrtPath, 1)
                q_path = rrtPath(j, :);
                if checkCollision(robot, q_path, collisionCylinders ,'SkippedSelfCollisions', 'parent')
                    totalPenalty = totalPenalty + collisionPenaltyValue;
                    break;
                end
            end
        end
        
        % Compute the achieved end-effector pose.
        T_ee = getTransform(robot, q_sol, robot.BodyNames{end});
        % Compute the relative transformation error.
        T_err = inv(T_goal) * T_ee;
        p_err = T_err(1:3,4);  % translation error.
        R_err = T_err(1:3,1:3); % rotation error.
        theta_err = acos(min(max((trace(R_err) - 1)/2, -1), 1));  % angular error.
        % Compute a unified pose error.
        pose_error = sqrt(norm(p_err)^2 + theta_err^2);
        totalPoseError = totalPoseError + pose_error;
        
        % Update the guess for the next goal.
        q_guess = q_sol;
    end
    
    % Weights for the different components.
    penaltyWeight = 1;
    designWeight  = 1;
    
    % Final objective combines pose error, design cost, and penalties.
    objective = totalPoseError + penaltyWeight * totalPenalty + designWeight * sum_design;
    
    fprintf('Objective: %.4f, Pose Error: %.4f, Penalty: %.4f, Design Sum: %.4f\n', ...
            objective, totalPoseError, penaltyWeight*totalPenalty, sum_design);
end
