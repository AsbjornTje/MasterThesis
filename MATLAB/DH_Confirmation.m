clear all;
clc;

% Define DH parameters for each joint.
% Each row is [a, alpha, d, theta] according to standard DH conventions.
% For a revolute joint the joint variable is theta; for a prismatic joint, d is variable.
dhparams = [
    0,-pi/2,0,0;
    0, -pi/2, 10, 0;   % Joint 1: prismatic (link length = 1, with a base offset d = 0.2)
    0.1, pi/2,   0.1, 0;   % Joint 2: revolute (link length = 1)
    0.5, 0,   0, 0;   % Joint 3: revolute (link length = 1)
];

% Specify the joint types for each joint.
% For example: joint 1 is revolute, joint 2 is prismatic, joint 3 is revolute.
jointTypes = ["fixed", "prismatic", "revolute", "revolute"];

% Create a rigidBodyTree robot model.
robot = rigidBodyTree("DataFormat", "row");

% Loop over each joint to create bodies and joints.
nJoints = size(dhparams,1);
for i = 1:nJoints
    % Extract the DH parameters for the i-th joint.
    a     = dhparams(i, 1);
    alpha = dhparams(i, 2);
    d     = dhparams(i, 3);
    theta = dhparams(i, 4);
    
    % Create a new rigid body with a unique name.
    body = rigidBody(sprintf("body%d", i));
    
    % Create the joint using the specified type.
    joint = rigidBodyJoint(sprintf("joint%d", i), jointTypes(i));
    
    % Set the fixed transform using the standard DH parameters.
    % The vector is provided as [a, alpha, d, theta].
    joint.setFixedTransform([a, alpha, d, theta], "dh");
    
    % Attach the joint to the body.
    body.Joint = joint;
    
    % Add the body to the robot:
    % For the first body, attach it to the base.
    % For subsequent bodies, attach to the previous body.
    if i == 1
        addBody(robot, body, robot.BaseName);
    else
        addBody(robot, body, sprintf("body%d", i-1));
    end
end

% Display the robot structure.
showdetails(robot);

% Set joint limits for the prismatic joint (which is link 1 in our SerialLink)
robot.getBody("body2").Joint.PositionLimits = [0, 1];

% % Visualize the robot in its home configuration.
% figure;
% % Use homeConfiguration (which returns a struct array) to get the default configuration.
config = homeConfiguration(robot);
% show(robot, config);
% title('3-Joint Robot with Mixed Revolute and Prismatic Joints');

% Create an interactive GUI (teach-pendant) for the robot.
gui = interactiveRigidBodyTree(robot, 'Configuration', config);
