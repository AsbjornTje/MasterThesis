%% corke_robot_script.m
% This script builds a robot using DH parameters via the Corke Robotics Toolbox,
% plots the robot in its home configuration, displays some environment cylinders,
% computes inverse kinematics for a set of goal poses, and animates joint-space trajectories.
% (Note: Collision checking and RRT planning from the original script are omitted.)

clear; clc; close all;

%% Define DH parameters and joint types
% Each row: [theta, d, a, alpha]
dhparams = [ ...
    pi/2  , 4    , 0.000 , -pi/2;   %Prismatic joint
    0     , 0.5  , 0     ,  pi/2;   % Row 2: revolute (yaw)
    0     , 0.1 , 0.500 , 0.000;   % Row 3: revolute (pitch)
    0 , 0.1    , 0.300 , 0.000;   % Row 4: revolute (pitch)
    ];

% Define joint types (first row is fixed; the moving joints are rows 2-5)
jointTypes = ["prismatic", "revolute", "revolute", "revolute"];

% Home configuration for the moving joints (order: row2, row3, row4, row5)
% (Matches your original: [0, pi/2, 0, 0])
q_home = [0, pi/2, 0, 0];

%% Create the robot using the Corke toolbox
robot = createRobotModel_Corke(dhparams, jointTypes, q_home);

% Set joint limits for the prismatic joint (which is link 1 in our SerialLink)
robot.qlim(1,:) = [-4, 0];

% Plot the robot in its home configuration
figure; robot.plot(q_home);
hold on;
title('Robot in Home Configuration');
axis equal; 
xlim([-10, 10]); ylim([-10, 10]); zlim([-10, 10]);

%% Generate and plot environment cylinders
% Define cylinders (as simple structures with radius and height)
collisionCylinders(1) = struct('radius', 0.457, 'height', 8.0);
collisionCylinders(2) = struct('radius', 0.044, 'height', 0.3);
collisionCylinders(3) = struct('radius', 0.044, 'height', 0.3);
collisionCylinders(4) = struct('radius', 0.350, 'height', 1.5);
collisionCylinders(5) = struct('radius', 0.350, 'height', 1.5);

% Each row: [x, y, z, ax, ay, az, angle]
cylinderPoses = [...
    -0.958, 4.260, -1.707+2, 1, 0, 0, pi/2; 
    -0.458, 2.015, -1.627+2, 1, 0, 0, pi/2;
    -1.456, 6.921, -1.627+2, 1, 0, 0, pi/2; 
    -0.952, 5.400, -0.986+2, 1, 0, 0, 0;
    -0.952, 6.900, -0.986+2, 1, 0, 0, 0
    ];

% Plot each cylinder using the helper function "plotCylinder"
for i = 1:length(collisionCylinders)
    pos   = cylinderPoses(i,1:3);
    axang = cylinderPoses(i,4:7);
    T = transl(pos) * axang2tform(axang);
    plotCylinder(collisionCylinders(i), T);
end

%% Define goal poses (each row: [x, y, z, ax, ay, az, angle])
goalPoses = [...
    -0.000, 1.700, 0.500, 0, 1, 0, pi/2;   % Pose 1 (start)
    -0.476, 2.015, -1.400+2, 0, 1, 0, pi/2;   % Pose 2 (front BLM)
    ];

disp('Plotting target poses...');
for i = 1:size(goalPoses,1)
    pos   = goalPoses(i,1:3);
    axang = goalPoses(i,4:7);
    T = transl(pos) * axang2tform(axang);
    trplot(T, 'frame', sprintf('Pose %d', i), 'color', 'r', 'length', 0.1);
    text(pos(1), pos(2), pos(3), sprintf('Pose %d', i), 'FontSize', 10, 'Color', 'r');
end

%% Inverse Kinematics and Trajectory Generation
% For our underactuated (4-DOF) robot we constrain only the position.
mask = [1 1 1 0 0 0];

numGoals = size(goalPoses,1);
qSolutions = zeros(robot.n, numGoals);
errorVectors = zeros(6, numGoals);
qInitial = q_home;

for i = 1:numGoals
    disp('Press Enter to move to the next pose...');
    pause;
    
    % Build desired end-effector transform from goal pose
    pos   = goalPoses(i,1:3);
    axang = goalPoses(i,4:7);
    targetTform = transl(pos) * axang2tform(axang);
    
    % Compute an inverse kinematics solution using the numerical method
    qSol = robot.ikine(targetTform, qInitial, mask);
    qSolutions(:, i) = qSol(:);
    
    % Generate an interpolated joint-space trajectory using jtraj
    numInterpSteps = 50;
    q_traj = jtraj(qInitial, qSol, numInterpSteps);
    
    % Animate the trajectory
    for k = 1:numInterpSteps
        robot.plot(q_traj(k,:));
        title(sprintf('Moving to Pose %d, step %d/%d', i, k, numInterpSteps));
        drawnow;
        pause(0.02);
    end
    
    % Compute the error vector at the final configuration.
    T_actual = robot.fkine(qSol);
    posDesired = transl(targetTform);
    posActual  = transl(T_actual);
    posError   = posDesired - posActual;
    
    % Compute orientation error (as an axis-angle error vector)
    R_desired = t2r(targetTform);
    R_actual  = t2r(T_actual);
    R_err     = R_actual' * R_desired;  % Relative rotation: R_actual*R_err = R_desired
    T_err     = [R_err, [0;0;0]; 0 0 0 1];
    [angle, ax, ay, az] = tr2angvec(T_err);
    orientError = [ax, ay, az] * angle;
    
    errorVector = [posError, orientError];
    errorVectors(:, i) = errorVector(:);
    
    fprintf('Pose %d error vector: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', ...
        i, errorVector(1), errorVector(2), errorVector(3), ...
        errorVector(4), errorVector(5), errorVector(6));
    
    % Update initial configuration for the next goal.
    qInitial = qSol;
end





