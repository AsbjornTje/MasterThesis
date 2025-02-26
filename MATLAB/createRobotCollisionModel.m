function [robot] = createRobotCollisionModel(dhparams, jointTypes, q_home)
%% Build the robot (rigidBodyTree)
numJoints = size(dhparams, 1);
bodies = cell(numJoints,1);
joints = cell(numJoints,1);
robot = rigidBodyTree("DataFormat", "row");

for i = 1:numJoints
    bodies{i} = rigidBody(['body' num2str(i)]);
    joints{i} = rigidBodyJoint(['jnt' num2str(i)], jointTypes(i));
    
    % Set the fixed transform using DH parameters.
    setFixedTransform(joints{i}, dhparams(i,:), "dh");
    bodies{i}.Joint = joints{i};
    
    % Attach body to tree.
    if i == 1
        addBody(robot, bodies{i}, "base");
    else
        addBody(robot, bodies{i}, bodies{i-1}.Name);
    end
end

%% Add Collision and Visual Objects
r = 0.07;  % default link radius
numBodies = numJoints; % number of bodies equals number of DH rows

% Loop over bodies starting from 3 (skip the base attached to "base")
for i = 3:numBodies
    if i == 5 || 6
        r = 0.061;
    end

    currentName = sprintf("body%d", i);
    parentName = sprintf("body%d", i-1);
    
    % Get the absolute values for a and d for the current body.
    h_a = abs(dhparams(i, 1));  % 'a' parameter (translation along x)
    h_d = abs(dhparams(i, 3));  % 'd' parameter (translation along z)
    
    % Get the transform from the parent body to the current body.
    T_current = getTransform(robot, q_home, parentName, currentName);
    
    % If h_d is nonzero, create a collision object using the d value.
    if h_d > 0
        T_d = T_current * trvec2tform([0, 0, h_d/2]);
        c_d = collisionCylinder(r, h_d, "Pose", T_d);
        addCollision(robot.getBody(currentName), c_d);
        addVisual(robot.getBody(currentName), "Cylinder", [r, h_d], T_d);
    end
    
    % If h_a is nonzero, create a collision object using the a value.
    if h_a > 0
        T_a = T_current * trvec2tform([h_a/2, 0, 0]) * axang2tform([0, 1, 0, -pi/2]);
        c_a = collisionCylinder(r, h_a, "Pose", T_a);
        addCollision(robot.getBody(currentName), c_a);
        addVisual(robot.getBody(currentName), "Cylinder", [r, h_a], T_a);
    end
end
end
