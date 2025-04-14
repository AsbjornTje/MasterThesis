function [robot, collisionData] = createRobotCollisionModel(dhparams, jointTypes, q_home)
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

%% Initialize collisionData structure.
collisionData = struct();

%% Add Collision and Visual Objects
defaultRadius = 0.055;  % default link radius
numBodies = numJoints; % number of bodies equals number of DH rows

% Loop over bodies starting from 3 (skip the base attached to "base")
for i = 3:numBodies
    % Set radius based on body index.
    if ismember(i, [5,6])
        r = 0.061;
    elseif ismember(i, [10, 11, 12])
        r = 0.055;
    else
        r = defaultRadius;
    end

    currentName = sprintf("body%d", i);
    parentName = sprintf("body%d", i-1);
    
    % Get the absolute values for a and d for the current body.
    a_val = dhparams(i, 1);
    d_val = dhparams(i, 3);  
    h_a = abs(a_val);
    h_d = abs(d_val);
    
    % Get the transform from the parent body to the current body at q_home.
    T_current = getTransform(robot, q_home, parentName, currentName);
    
    % Create a temporary container for collision data for this body.
    collisionDataCell = {};
    
    if (h_a > 0) && (h_d > 0)
        % Both parameters are nonzero.
        if h_a >= h_d
            % 'a' is longer: shift the a-cylinder along the z-direction using d_val.
            T_a = T_current * trvec2tform([h_a/2, 0, d_val]) * axang2tform([0, 1, 0, -pi/2]);
            % For the d-cylinder, use the signed value.
            T_d = T_current * trvec2tform([0, 0, d_val/2]);
        else
            % d is longer: modify the d-cylinder.
            T_d = T_current * trvec2tform([h_a, 0, d_val/2]);
            T_a = T_current * trvec2tform([h_a/2, 0, 0]) * axang2tform([0, 1, 0, -pi/2]);
        end
        
        % Create collision and visual objects for the d-cylinder.
        c_d = collisionCylinder(r, h_d, "Pose", T_d);
        addCollision(robot.getBody(currentName), c_d);
        addVisual(robot.getBody(currentName), "Cylinder", [r, h_d], T_d);
        % Store the parameters (store the global pose as computed).
        data_d.Radius = r;
        data_d.Length = h_d;
        data_d.Pose = T_d;
        collisionDataCell{end+1} = data_d;
        
        % Create collision and visual objects for the a-cylinder.
        c_a = collisionCylinder(r, h_a, "Pose", T_a);
        addCollision(robot.getBody(currentName), c_a);
        addVisual(robot.getBody(currentName), "Cylinder", [r, h_a], T_a);
        data_a.Radius = r;
        data_a.Length = h_a;
        data_a.Pose = T_a;
        collisionDataCell{end+1} = data_a;
        
    else
        % If only one parameter is nonzero, handle each separately.
        if h_d > 0
            T_d = T_current * trvec2tform([0, 0, d_val/2]);
            c_d = collisionCylinder(r, h_d, "Pose", T_d);
            addCollision(robot.getBody(currentName), c_d);
            addVisual(robot.getBody(currentName), "Cylinder", [r, h_d], T_d);
            data_d.Radius = r;
            data_d.Length = h_d;
            if d_val < 0
                data_d.Pose = T_d * trvec2tform([0, 0, d_val/2]);
            else
                data_d.Pose = T_d * inv(trvec2tform([0, 0, d_val/2]));
            end
            collisionDataCell{end+1} = data_d;
        end
        if h_a > 0
            T_a = T_current * trvec2tform([h_a/2, 0, 0]) * axang2tform([0, 1, 0, -pi/2]);
            c_a = collisionCylinder(r, h_a, "Pose", T_a);
            addCollision(robot.getBody(currentName), c_a);
            addVisual(robot.getBody(currentName), "Cylinder", [r, h_a], T_a);
            data_a.Radius = r;
            data_a.Length = h_a;
            data_a.Pose = T_a * inv(trvec2tform([0, 0, h_a/2]));
            collisionDataCell{end+1} = data_a;
        end
    end
    % Store collision data for this body in the collisionData structure.
    collisionData.(currentName) = collisionDataCell;
end
end
