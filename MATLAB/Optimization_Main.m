clear all;
clc; 
%close all;

% Specify DH parameters
dhparams_full = [
    % a       alpha    d     theta
       0     -pi/2    -0.306      0;
       0     -pi/2   10      0;
       0      pi/2   0.108     0;
       0     -pi/2   0.2     0;
       0      0     1.37     pi/2; %change this cylinder diameter to 0.061
       0.2    0     0     0; % and this one
       0     -pi/2    0.2    0; %min 0.194 max 0.6
       0      pi/2    0.2     0; %min 0.81 max 0.15
       0     -pi/2    0.2    0; %min 0.14 max 0.6
       0      pi/2    0.2    0; %min 0.081 max 0.15
       0        0      0.2    0; % min 0.14 max 0.6
];

% Define joint types and home configuration
rev = "revolute";
pris = "prismatic";
fix = "fixed";
jointTypes = [fix, pris, rev, rev, fix, pris, rev, rev, rev, rev, rev];
q_home = [0 0 0 0 0 0 0 0 0];


% Create a rigidBodyTree robot model.
robot = createRobotCollisionModel(dhparams_full, jointTypes, q_home)

robot.getBody("body2").Joint.PositionLimits = [0, 10];
robot.getBody("body6").Joint.PositionLimits = [-1, 0];

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

% show(robot,q_home,"collisions","on")
% 
% % Wait for user input
% disp('Press Enter to move to the first pose...');
% pause;

% Define goal poses
goalPoses = [
    -0.476, 2.015, -1.400, 1, 0, 0, -pi;  % Front BLM
    -0.476, 6.400, -1.200, 1, 0, 0, -pi;
    -1.469, 6.921, -1.400, 1, 0, 0, -pi;  % Back BLM
];

% Define obstacle environment
collisionCylinders = {
    collisionCylinder(0.457, 8.0)...  % Main Pipe
    collisionCylinder(0.044, 0.3)...  % Front BLM
    collisionCylinder(0.044, 0.3)...   % Back BLM
    collisionCylinder(0.350, 1.5)...  % First Obstacle
    collisionCylinder(0.350, 1.5)...   % Second Obstacle
};

collisionCylinders{1}.Pose = trvec2tform([-0.958, 4.260, -1.707]) * axang2tform([1, 0, 0, pi/2]);
collisionCylinders{2}.Pose = trvec2tform([-0.458, 2.015, -1.627]) * axang2tform([1, 0, 0, pi/2]);
collisionCylinders{3}.Pose = trvec2tform([-1.456, 6.921, -1.627]) * axang2tform([1, 0, 0, pi/2]);
collisionCylinders{4}.Pose = trvec2tform([-0.952, 5.400, -0.6]) * axang2tform([1, 0, 0, 0]);
collisionCylinders{5}.Pose = trvec2tform([-0.952, 6.900, -0.6]) * axang2tform([1, 0, 0, 0]);

% Define original variables 
originalVars = table();
originalVars.a6 = dhparams_full(6,1);
originalVars.d7 = dhparams_full(7,3);
originalVars.d8 = dhparams_full(8,3);
originalVars.d9 = dhparams_full(9,3);
originalVars.d10 = dhparams_full(10,3);
originalVars.d11 = dhparams_full(11,3);

% Define optimization variables
optVars = [ optimizableVariable('a6', [0.2, 0.3], 'Type', 'real'), ...
            optimizableVariable('d7', [0.2, 0.9], 'Type', 'real'), ...
            optimizableVariable('d8', [0.2, 0.3], 'Type', 'real'), ...
            optimizableVariable('d9', [0.2, 0.9], 'Type', 'real'), ...
            optimizableVariable('d10', [0.2, 0.3], 'Type', 'real'), ...
            optimizableVariable('d11', [0.2, 0.9], 'Type', 'real')];

objFcn = @(x) ObjectiveFcn_Collision(x, dhparams_full, jointTypes, goalPoses, q_home);

%% Run Bayesian Optimization
results = bayesopt(objFcn, optVars, ...
    'MaxObjectiveEvaluations', 200, ...
    'IsObjectiveDeterministic', true, ...
    'AcquisitionFunctionName', 'expected-improvement-plus', ...
    'Verbose', 1 ...
    );

% Extract best parameters
disp('Original design variables (for joints 6 and up):');
disp(originalVars);
bestVars = bestPoint(results);
disp('Best design variables:');
disp(bestVars);

%% Update DH parameters using optimized design variables
dhparams_opt = dhparams_full;
dhparams_opt(6,1)  = bestVars.a6;
dhparams_opt(7,3)  = bestVars.d7;
dhparams_opt(8,3)  = bestVars.d8;
dhparams_opt(9,3)  = bestVars.d9;
dhparams_opt(10,3) = bestVars.d10;
dhparams_opt(11,3) = bestVars.d11;

disp('Updated DH Parameters:');
disp(dhparams_opt);

% Create the optimized robot model
robot_opt = createRobotCollisionModel(dhparams_opt, jointTypes, q_home);

robot_opt.getBody("body2").Joint.PositionLimits = [0, 10];
robot_opt.getBody("body6").Joint.PositionLimits = [-1, 0];

% setup visualisation
figure;
ax = axes;
show(robot_opt, q_home, "Collisions", "on", 'Parent', ax);
title(ax, 'Optimized Robot - Home Configuration');
view(ax, 3);
axis(ax, [-3 3 -2 10 -3 1]);
hold(ax, 'on');

% Plot collision objects on the same axes
for i = 1:length(collisionCylinders)
    show(collisionCylinders{i}, 'Parent', ax);
end


% Plot goal poses on the same axes
for i = 1:size(goalPoses, 1)
    pos = goalPoses(i, 1:3);
    rot = goalPoses(i, 4:7);
    poseTF = trvec2tform(pos) * axang2tform(rot);
    plotTransforms(pos, tform2quat(poseTF), 'FrameSize', 0.1, 'Parent', ax);
    text(ax, pos(1), pos(2), pos(3), sprintf('Pose %d', i), 'FontSize', 10, 'Color', 'r');
end

% Wait for user input
disp('Press any key to start path planning...');
pause;

%Set up 
gik = generalizedInverseKinematics('RigidBodyTree', robot_opt, ...
                                   'ConstraintInputs', {'pose', 'joint'});

jointBounds = constraintJointBounds(robot_opt);
jointBounds.Bounds = jointLimits;

planner = manipulatorRRT(robot_opt,collisionCylinders);
planner.SkippedSelfCollisions = 'parent';
planner.EnableConnectHeuristic = false;
planner.MaxConnectionDistance = 0.1;
planner.ValidationDistance = 0.01;
planner.MaxIterations = 10000;
rng(0)

q_guess = q_home;  % Start from the home configuration

for i = 1:size(goalPoses, 1)
    % Define the pose constraint for the current goal pose.
    poseTgt = constraintPoseTarget(robot_opt.BodyNames{end});
    poseTgt.TargetTransform = trvec2tform(goalPoses(i, 1:3)) * axang2tform(goalPoses(i, 4:7));
    
    % Solve the constrained IK problem using the current guess.
    [q_sol, solInfo] = gik(q_guess, poseTgt, jointBounds);
    
    % If IK fails, you may wish to handle that (e.g., skip this pose or add a penalty).
    if solInfo.Status ~= "success"
        warning('IK failed for goal %d. Skipping trajectory planning for this goal.', i);
        q_guess = q_sol;  % update guess and continue
        continue;
    end

    % Pre-check the goal configuration for collision.
    if checkCollision(robot_opt, q_sol, collisionCylinders, 'SkippedSelfCollision', 'parent')
        warning('Goal configuration for pose %d is in collision. Skipping trajectory planning for this goal.', i);
        q_guess = q_sol;
        continue;
    end
    
    % Plan a collision-free trajectory using the planner object.
    try
        collisionFreePath = planner.plan(q_guess, q_sol);
    catch ME
        warning('RRT planning failed for goal %d: %s', i, ME.message);
        collisionFreePath = [];
    end
    
    if isempty(collisionFreePath)
        warning('No collision-free path found for pose %d.', i);
    else
        % Animate the collision-free trajectory and check for collisions with obstacles.
        for j = 1:size(collisionFreePath, 1)
            q_traj = collisionFreePath(j,:);
            if checkCollision(robot_opt, q_traj, collisionCylinders, 'SkippedSelfCollision', 'parent')
                warning('Collision detected along trajectory at configuration %d for pose %d.', j, i);
            end
            show(robot_opt, q_traj, 'collisions', 'on', 'Parent', ax, 'PreservePlot', false);
            title(ax, sprintf('Moving to Goal Pose %d', i));
            drawnow;
            pause(0.05);
        end
    end
    
    % Update the starting configuration for the next segment.
    q_guess = q_sol;
    
    disp('Press Enter to move to the next pose...');
    pause;
end

close all;
clear all;
