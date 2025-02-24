clear all;
clc;

% Specify DH parameters
dhparams = [
    % a       alpha    d     theta
       0     -pi/2    -0.306      0;
       0     -pi/2   10      0;
       0      pi/2   0.108     0;
       0     -pi/2   0.061     0;
       0      0     1.37     pi/2; %change this cylinder diameter to 0.061
       0.097    0     0     0; % and this one
       0     -pi/2    0.5    0; %min 0.194 max 0.6
       0      pi/2    0.1     0; %min 0.81 max 0.15
       0     -pi/2    0.5    0; %min 0.14 max 0.6
       0      pi/2    0.1    0; %min 0.081 max 0.15
       0        0      0.5    0; % min 0.14 max 0.6
];

% Define joint types and home configuration
rev = "revolute";
pris = "prismatic";
fix = "fixed";
jointTypes = [fix, pris, rev, rev, fix, pris, rev, rev, rev, rev, rev];
q_home = [0 0 0 0 0 0 0 0 0];


% Create a rigidBodyTree robot model.
robot = createRobotModel_Generalized(dhparams, jointTypes, q_home)

robot.getBody("body2").Joint.PositionLimits = [0, 10];
robot.getBody("body6").Joint.PositionLimits = [-1, 0];

% % Visualize the robot in its home configuration.
% figure;
% % Use homeConfiguration (which returns a struct array) to get the default configuration.
config = homeConfiguration(robot);
% show(robot, config);
% title('3-Joint Robot with Mixed Revolute and Prismatic Joints');

% Create an interactive GUI (teach-pendant) for the robot.
gui = interactiveRigidBodyTree(robot, 'Configuration', config);
