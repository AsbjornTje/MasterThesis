function [robot] = createRobotModel_Generalized(dhparams, jointTypes, q_home)
% createRobotModel_Generalized Constructs a rigidBodyTree robot using input DH parameters.
%
%   [robot] = createRobotModel_Generalized(dhparams, jointTypes, q_home) builds a
%   robot model from the provided DH parameter matrix and joint types, and attaches
%   collision and visual geometry based on the DH parameters.
%
%   The DH parameter matrix (dhparams) is an N-by-4 matrix with rows:
%       [a, alpha, d, theta]
%
%   For each body (except the base), this function checks:
%     - If the d parameter (translation along z) is nonzero, it creates a cylinder
%       with height = |d| and attaches it along the z-axis.
%     - If the a parameter (translation along x) is nonzero, it creates a cylinder
%       with height = |a| and attaches it along the x-axis (by applying a -90° rotation
%       about the y-axis).
%
%   The output robot is a rigidBodyTree with the collision and visual geometries attached.
%
%   Example usage:
%       robot = createRobotModel_Generalized(dhparams, jointTypes, q_home);

    %% Build the robot (rigidBodyTree)
    
    numJoints = size(dhparams, 1);
    bodies = cell(numJoints,1);
    joints = cell(numJoints,1);
    robot = rigidBodyTree("DataFormat", "row");
    
    for i = 1:numJoints
        bodies{i} = rigidBody(['body' num2str(i)]);
        joints{i} = rigidBodyJoint(['jnt' num2str(i)], jointTypes(i));
        
        % Set the fixed transform using DH parameters
        setFixedTransform(joints{i}, dhparams(i,:), "dh");
        bodies{i}.Joint = joints{i};
        
        % Attach body to tree
        if i == 1
            addBody(robot, bodies{i}, "base");
        else
            addBody(robot, bodies{i}, bodies{i-1}.Name);
        end
    end

    %% Add Collision and Visual Objects
    r = 0.07;  % default link radius
    numBodies = numJoints; % number of bodies equals number of DH rows

    % Loop over bodies starting from 2 (skip the base attached to "base")
    for i = 3:numBodies
        % Form names for parent and current body
        parentName = sprintf("body%d", i-1);
        currentName = sprintf("body%d", i);
       
       % Get the absolute values for a and d for the current body.
    h_a = abs(dhparams(i, 1));  % 'a' parameter (translation along x)
    h_d = abs(dhparams(i, 3));  % 'd' parameter (translation along z)
    
    % Get the transform from the parent body to the current body.
    T_current = getTransform(robot,q_home, parentName, currentName);
    
    if (h_a > 0) && (h_d > 0)
        % Both parameters are nonzero. We want to leave the cylinder
        % corresponding to the shorter link unchanged and shift the center of the longer link.
        if h_a >= h_d
            % 'a' is longer, so we modify the a-cylinder.
            % The standard placement is at [a/2, 0, 0]. We now add an extra translation along the z-axis (d-direction)
            % equal to h_d.
            T_a = T_current * trvec2tform([h_a/2, 0, h_d]) * axang2tform([0, 1, 0, -pi/2]);
            % The d-cylinder remains in its usual place.
            T_d = T_current * trvec2tform([0, 0, h_d/2]);
        else
            % h_d > h_a: d is longer, so we modify the d-cylinder.
            % The standard placement for d is [0, 0, d/2]. Now, shift along the x-axis (a-direction)
            % by h_a.
            T_d = T_current * trvec2tform([h_a, 0, h_d/2]);
            % The a-cylinder remains in its usual placement.
            T_a = T_current * trvec2tform([h_a/2, 0, 0]) * axang2tform([0, 1, 0, -pi/2]);
        end
        
        % Create collision and visual objects for both geometries.
        c_d = collisionCylinder(r, h_d, "Pose", T_d);
        addCollision(robot.getBody(currentName), c_d);
        addVisual(robot.getBody(currentName), "Cylinder", [r, h_d], T_d);
        
        c_a = collisionCylinder(r, h_a, "Pose", T_a);
        addCollision(robot.getBody(currentName), c_a);
        addVisual(robot.getBody(currentName), "Cylinder", [r, h_a], T_a);
        
    else
        % If only one parameter is nonzero, use the original placements.
        if h_d > 0
            T_d = T_current * trvec2tform([0, 0, h_d/2]);
            c_d = collisionCylinder(r, h_d, "Pose", T_d);
            addCollision(robot.getBody(currentName), c_d);
            addVisual(robot.getBody(currentName), "Cylinder", [r, h_d], T_d);
        end
        if h_a > 0
            T_a = T_current * trvec2tform([h_a/2, 0, 0]) * axang2tform([0, 1, 0, -pi/2]);
            c_a = collisionCylinder(r, h_a, "Pose", T_a);
            addCollision(robot.getBody(currentName), c_a);
            addVisual(robot.getBody(currentName), "Cylinder", [r, h_a], T_a);
        end
    end
    end
end