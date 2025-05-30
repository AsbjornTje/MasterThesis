clear all;
clc; 
%close all;

% Specify DH parameters
dhparams_full = [
    % a       alpha    d     theta
       0     -pi/2    0      0;
       0     -pi/2   10      0;
       0      pi/2   0.2     0;
       0     -pi/2   0.2     0;
       0      pi/2   1     0;
       0     -pi/2   0.2     0;
       0      pi/2   0.2     0;
       0     -pi/2   0.2     0;
       0      pi/2   0.1     0;
];

% Define joint types and home configuration
rev = "revolute";
pris = "prismatic";
jointTypes = [rev, pris, rev, rev, pris, rev, rev, rev, rev];
q_home = [0 0 0 0 0.5 0 0 0 0];

% Create robot model
robot = createRobotModel_Corke(dhparams_full, jointTypes);

% Define joint limits
jointLimits = [
    0, 0;   % Joint 1
    0, 10;     % Joint 2
    -pi, pi;   % Joint 3
    -pi, pi;   % Joint 4
    0, 1.4;    % Joint 5
    -pi, pi;   % Joint 6
    -pi, pi;   % Joint 7
    -pi, pi;   % Joint 8
    -pi, pi;   % Joint 9
];

robot.getBody("body2").Joint.PositionLimits = [0, 10];
robot.getBody("body1").Joint.PositionLimits = [0, 0];
robot.getBody("body5").Joint.PositionLimits = [0, 1.4];

% Define goal poses
goalPoses = [
    -0.476, 2.015, -1.400, 1, 0, 0, -pi/2;  % Front BLM
    -1.469, 6.921, -1.400, 1, 0, 0, -pi/2;  % Back BLM
];

% Define original variables 
originalVars = table();
for i = 6:size(dhparams_full, 1)
    originalVars.(sprintf('a%d', i)) = dhparams_full(i,1);
    originalVars.(sprintf('d%d', i)) = dhparams_full(i,3);
end

% Define optimization variables
optVars = [];
for i = 6:size(dhparams_full, 1)
    a_var = optimizableVariable(sprintf('a%d', i), [0, 0.2], 'Type', 'real');
    d_var = optimizableVariable(sprintf('d%d', i), [0.2, 1], 'Type', 'real');
    optVars = [optVars, a_var, d_var];
end

objFcn = @(x) objectiveFcn(x, dhparams_full, jointTypes, goalPoses, q_home);

% Define stopping thresholds
threshold_obj = 0.1 + 0.05; % translation + rotation threshold

%% Run Bayesian Optimization
results = bayesopt(objFcn, optVars, ...
    'MaxObjectiveEvaluations', 50, ...
    'IsObjectiveDeterministic', true, ...
    'AcquisitionFunctionName', 'expected-improvement-plus', ...
    'Verbose', 1);

% Extract best parameters
disp('Original design variables (for joints 6 and up):');
disp(originalVars);
bestVars = bestPoint(results);
disp('Best design variables:');
disp(bestVars);

%% Update DH parameters using optimized design variables
dhparams_opt = dhparams_full;
for i = 6:size(dhparams_full, 1)
    dhparams_opt(i,1) = bestVars.(sprintf('a%d', i));
    dhparams_opt(i,3) = bestVars.(sprintf('d%d', i));
end

% Create the optimized robot model
robot_opt = createRobotModel_Corke(dhparams_opt, jointTypes);

robot_opt.getBody("body2").Joint.PositionLimits = [0, 10];
robot_opt.getBody("body1").Joint.PositionLimits = [0, 0];
robot_opt.getBody("body5").Joint.PositionLimits = [0, 1.4];

% setup visualisation
figure;
ax = axes;
show(robot, q_home, 'Parent', ax);
title(ax, 'Non Optimized Robot - Home Configuration');
view(ax, 3);
axis(ax, [-3 3 -2 10 -3 1]);
hold(ax, 'on');

% Add Collsion objects
collisionCylinders = {
    collisionCylinder(0.457, 8.0),  % Main Pipe
    collisionCylinder(0.044, 0.3),  % Front BLM
    collisionCylinder(0.044, 0.3)   % Back BLM
};

cylinderPoses = [
    -0.958, 4.260, -1.707, 1, 0, 0, pi/2; 
    -0.458, 2.015, -1.627, 1, 0, 0, pi/2;
    -1.456, 6.921, -1.627, 1, 0, 0, pi/2; 
];

for i = 1:length(collisionCylinders)
    P = trvec2tform(cylinderPoses(i, 1:3));
    R = axang2tform(cylinderPoses(i, 4:7));
    collisionCylinders{i}.Pose = P * R;
    show(collisionCylinders{i});
end

% Plot goal poses
for i = 1:size(goalPoses, 1)
    pos = goalPoses(i, 1:3);
    rot = goalPoses(i, 4:7);
    poseTF = trvec2tform(pos) * axang2tform(rot);
    plotTransforms(pos, tform2quat(poseTF), "FrameSize", 0.1);
    text(pos(1), pos(2), pos(3), sprintf('Pose %d', i), 'FontSize', 10, 'Color', 'r');
end

% Wait for user input
disp('Press Enter to move to the first pose...');
pause;

%Set up 
gik = generalizedInverseKinematics('RigidBodyTree', robot, ...
                                   'ConstraintInputs', {'pose', 'joint'});

jointBounds = constraintJointBounds(robot);
jointBounds.Bounds = jointLimits;

q_guess = q_home;  % initialize initial guess

for i = 1:size(goalPoses, 1)
    % Define pose constraint for current goal pose.
    poseTgt = constraintPoseTarget(robot.BodyNames{end});
    poseTgt.TargetTransform = trvec2tform(goalPoses(i, 1:3)) * axang2tform(goalPoses(i, 4:7));
    
    % Solve the constrained IK problem using the current guess.
    [q_sol, solInfo] = gik(q_guess, poseTgt, jointBounds);
    
    % Update the initial guess for the next iteration.
    q_guess = q_sol;
    
    % Update robot display.
    show(robot, q_sol, 'Parent', ax, 'PreservePlot', false);
    title(ax, sprintf('Non Optimized Robot - Goal Pose %d', i));
    drawnow;
    
    % Wait for user input.
    disp('Press Enter to move to the next pose...');
    pause;
end

hold off;

% setup visualisation
figure;
ax = axes;
show(robot_opt, q_home, 'Parent', ax);
title(ax, 'Optimized Robot - Home Configuration');
view(ax, 3);
axis(ax, [-3 3 -2 10 -3 1]);
hold(ax, 'on');

% Add Collsion objects
collisionCylinders = {
    collisionCylinder(0.457, 8.0),  % Main Pipe
    collisionCylinder(0.044, 0.3),  % Front BLM
    collisionCylinder(0.044, 0.3)   % Back BLM
};

cylinderPoses = [
    -0.958, 4.260, -1.707, 1, 0, 0, pi/2; 
    -0.458, 2.015, -1.627, 1, 0, 0, pi/2;
    -1.456, 6.921, -1.627, 1, 0, 0, pi/2; 
];

for i = 1:length(collisionCylinders)
    P = trvec2tform(cylinderPoses(i, 1:3));
    R = axang2tform(cylinderPoses(i, 4:7));
    collisionCylinders{i}.Pose = P * R;
    show(collisionCylinders{i});
end

% Plot goal poses
for i = 1:size(goalPoses, 1)
    pos = goalPoses(i, 1:3);
    rot = goalPoses(i, 4:7);
    poseTF = trvec2tform(pos) * axang2tform(rot);
    plotTransforms(pos, tform2quat(poseTF), "FrameSize", 0.1);
    text(pos(1), pos(2), pos(3), sprintf('Pose %d', i), 'FontSize', 10, 'Color', 'r');
end

% Wait for user input
disp('Press Enter to move to the first pose...');
pause;

%Set up 
gik = generalizedInverseKinematics('RigidBodyTree', robot_opt, ...
                                   'ConstraintInputs', {'pose', 'joint'});

jointBounds = constraintJointBounds(robot_opt);
jointBounds.Bounds = jointLimits;

q_guess = q_home;  % initialize initial guess

for i = 1:size(goalPoses, 1)
    % Define pose constraint for current goal pose.
    poseTgt = constraintPoseTarget(robot_opt.BodyNames{end});
    poseTgt.TargetTransform = trvec2tform(goalPoses(i, 1:3)) * axang2tform(goalPoses(i, 4:7));
    
    % Solve the constrained IK problem using the current guess.
    [q_sol, solInfo] = gik(q_guess, poseTgt, jointBounds);
    
    % Update the initial guess for the next iteration.
    q_guess = q_sol;
    
    % Update robot display.
    show(robot_opt, q_sol, 'Parent', ax, 'PreservePlot', false);
    title(ax, sprintf('Optimized Robot - Goal Pose %d', i));
    drawnow;
    
    % Wait for user input.
    disp('Press Enter to move to the next pose...');
    pause;
end

% close all;
% clear all;
