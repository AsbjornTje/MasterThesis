function showOptimizedRobot(dhparams, jointTypes, q_home, bestVars)
    %% Setup the robot model and goal poses
    % Clear workspace and setup
    clc; close all;
    
    % Specify DH parameters (each row: [a, alpha, d, theta])
    dhparams_full = dhparams;
    
    % Define joint types and home configuration
    jointTypes = jointTypes
    q_home = q_home
    
    % Create robot and set limits
    robot = createRobotModel_Corke(dhparams_full,jointTypes);
    robot.getBody("body2").Joint.PositionLimits = [0, 10];
    robot.getBody("body1").Joint.PositionLimits = [0, 0];
    robot.getBody("body5").Joint.PositionLimits = [0, 1.4];
    
    % Define goal poses [x y z ax ay az angle]
    goalPoses = [
        -0.476, 2.015, -1.400, 1, 0, 0, -pi/2;
        -0.476, 6.200, -1.400, 1, 0, 0, -pi/2;
        -0.900, 6.300, -1.000, 1, 0, 0, -pi/2;
        -1.469, 6.400, -1.400, 1, 0, 0, -pi/2;
        -1.469, 6.921, -1.400, 1, 0, 0, -pi/2;
    ];
    
    %% Assume bestVars is available (from your bayesopt run)
    % For this example, we'll assume bestVars has been loaded into the workspace.
    % If not, load it using: load('bestVars.mat');
    if ~exist('bestVars', 'var')
        error('bestVars not found. Please run the optimization or load bestVars.');
    end
    
    % Update the DH parameters with optimized values for joints 6:end.
    dhparams_opt = dhparams_full;
    for i = 6:size(dhparams_full, 1)
        field_a = sprintf('a%d', i);
        field_d = sprintf('d%d', i);
        dhparams_opt(i,1) = bestVars.(field_a);
        dhparams_opt(i,3) = bestVars.(field_d);
    end
    
    % Create the optimized robot model.
    robot = createRobotModel_Corke(dhparams_opt, jointTypes);
    
    %% Create inverse kinematics solver
    ik = inverseKinematics('RigidBodyTree', robot);
    weights = [0.25 0.25 0.25 1 1 1]; % weights for [x y z roll pitch yaw]
    initialguess = q_home;
    
    %% Setup figure for visualization
    fig = figure;
    ax = axes('Parent', fig);
    show(robot, q_home, 'Parent', ax);
    title(ax, 'Optimized Robot - Home Configuration');
    view(ax, 3); axis(ax, 'equal'); hold(ax, 'on');
    
    % Plot goal pose frames for reference
    for i = 1:size(goalPoses, 1)
        pos = goalPoses(i, 1:3);
        rot = goalPoses(i, 4:7);
        T_goal = trvec2tform(pos) * axang2tform(rot);
        plotTransforms(ax, pos, tform2quat(T_goal), 'FrameSize', 0.1);
        text(pos(1), pos(2), pos(3), sprintf('Pose %d', i), 'FontSize', 10, 'Color', 'r');
    end
    
    %% Shared variable for current goal pose index
    currentGoal = 1;
    
    % Create a push button to cycle through goal poses
    btn = uicontrol('Style', 'pushbutton', 'String', 'Next Goal Pose',...
        'Position', [20 20 120 30],...
        'Callback', @nextGoalCallback);
    
    %% Nested callback function: can access currentGoal and other variables
    function nextGoalCallback(~,~)
    % Get the current goal pose
    pos_goal = goalPoses(currentGoal,1:3);
    axang_goal = goalPoses(currentGoal,4:7);
    T_goal = trvec2tform(pos_goal) * axang2tform(axang_goal);
    
    % Compute the IK solution for the end-effector (last body) to reach T_goal.
    [q_sol, solInfo] = ik(robot.BodyNames{end}, T_goal, weights, initialguess);
    initialguess = q_sol;  % update initial guess
    
    % Update the robot display with the new configuration.
    show(robot, q_sol, 'Parent', ax, 'PreservePlot', false);
    title(ax, sprintf('Optimized Robot - Goal Pose %d', currentGoal));
    drawnow;
    
    % Plot the goal frame ensuring the quaternion is 1x4.
    quatGoal = tform2quat(T_goal);
    if size(quatGoal,1) > size(quatGoal,2)
        quatGoal = quatGoal';  % transpose to get a row vector
    end
    plotTransforms(ax, pos_goal, quatGoal, "FrameSize", 0.1);
    
    % Cycle to the next goal pose
    currentGoal = currentGoal + 1;
    if currentGoal > size(goalPoses,1)
        currentGoal = 1;
    end
end


end
