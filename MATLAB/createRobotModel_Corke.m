function robot = createRobotModel_Corke(dhparams, jointTypes)
    % createRobotModelFromDH creates a robot model using standard DH parameters.
    %
    % Inputs:
    %   dhparams   - an n x 4 matrix where each row is [a, alpha, d, theta]
    %   jointTypes - an array (or cell array) of strings of length n; each element is 
    %                either 'revolute' or 'prismatic'
    %
    % Output:
    %   robot - a rigidBodyTree robot model constructed from the DH parameters.
    %
    % Example:
    %   dhparams = [0, -pi/2, 0, 0;
    %               0, -pi/2, 10, 0;
    %               0,  pi/2, 0.1, 0;
    %               0, -pi/2, 0.1, 0;
    %               0,  pi/2, 1.5, 0;
    %               0, -pi/2, 0.1, 0;
    %               0,  pi/2, 1, 0];
    %   jointTypes = ["revolute", "prismatic", "revolute", "revolute", ...
    %                 "prismatic", "revolute", "revolute"];
    %   robot = createRobotModelFromDH(dhparams, jointTypes);
    
    numJoints = size(dhparams, 1);
    robot = rigidBodyTree("DataFormat", "row", "MaxNumBodies", numJoints);
    parentName = robot.BaseName;
    
    for i = 1:numJoints
        a     = dhparams(i, 1);
        alpha = dhparams(i, 2);
        d     = dhparams(i, 3);
        theta = dhparams(i, 4);
        
        % Create a body and its corresponding joint
        body = rigidBody(sprintf("body%d", i));
        joint = rigidBodyJoint(sprintf("joint%d", i), jointTypes(i));
        
        % Set the fixed transform for this joint using the DH parameters.
        % Note: the "dh" option specifies that the transformation is interpreted 
        % according to the standard Denavit-Hartenberg convention.
        setFixedTransform(joint, [a, alpha, d, theta], "dh");
        body.Joint = joint;
        
        % Add the body to the robot, attaching it to the previous body.
        addBody(robot, body, parentName);
        parentName = body.Name;
    end
end
