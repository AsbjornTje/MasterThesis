function [robot] = createRobotModel(dhparams, jointTypes, q_home)
% createRobotModel Constructs a rigidBodyTree robot using input DH parameters.
%
%   [robot, q] = createRobotModel(dhparams) builds a robot model from the
%   provided DH parameter matrix. The input dhparams is a Nx4 matrix,
%   where each row contains the DH parameters in the order:
%       [a, alpha, d, theta]
%
%   The function also adds collision and visual geometry to the robot.
%
%   Outputs:
%       robot - The constructed rigidBodyTree object.

    %% Build the robot (rigidBodyTree)
    q = q_home;
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

    % body 1 
    h1 = abs(dhparams(1,3));
    t01 = getTransform(robot, q, "base", "body1") * trvec2tform([0 0 h1/2]);
    c1 = collisionCylinder(r,h1, "Pose", t01);
    addCollision(robot.getBody("body1"), c1);
    addVisual(robot.getBody("body1"), "Cylinder", [r h1], t01);

    % --- Body 2 ---
    % This is the body simulating the wagon and does not have any collision
    % object for simplification. It is simply a prismatic joint with no
    % collision to simulate the movement along the ceiling mounted rail.

    % --- New Block for Body3 (shifted from previous Body2) ---
    h3 = abs(dhparams(3,3));
    t03 = getTransform(robot, q, "body2", "body3") * trvec2tform([0 0 -h3/2]);
    c3 = collisionCylinder(r, h3, "Pose", t03);
    addCollision(robot.getBody("body3"), c3);
    addVisual(robot.getBody("body3"), "Cylinder", [r h3], t03);
    
    % --- New Block for Body4 (shifted from previous Body3) ---
    h4 = abs(dhparams(4,3));
    t04 = getTransform(robot, q, "body3", "body4") * trvec2tform([0 0 -h4/2]);
    c4 = collisionCylinder(r, h4, "Pose", t04);
    addCollision(robot.getBody("body4"), c4);
    addVisual(robot.getBody("body4"), "Cylinder", [r h4], t04);
    
    % --- New Block for Body5 (shifted from previous Body4) ---
    % (Preserving the special scaling from your original code, if any)
    h5 = abs(dhparams(5,3)) * 2;
    t05 = getTransform(robot, q, "body4", "body5") * trvec2tform([0 0 h5/2]);
    c5 = collisionCylinder(0.065, h5, "Pose", t05);
    addCollision(robot.getBody("body5"), c5);
    addVisual(robot.getBody("body5"), "Cylinder", [0.065 h5], t05);
    
    % --- New Block for Body7 (shifted from previous Body6) ---
    % w7 = abs(dhparams(7,1));
    % h7 = abs(dhparams(7,3));
    % t07 = getTransform(robot, q, "body5", "body7") * trvec2tform([w7 0 h7]);
    % c7 = collisionCylinder(r, h7, "Pose", t07);
    % addCollision(robot.getBody("body7"), c7);
    % %addVisual(robot.getBody("body7"), "Cylinder", [r h7], t07);
    
    % --- New Block for Body8 (shifted from previous Body7) ---
    h8 = abs(dhparams(8,3));
    t08 = getTransform(robot, q, "body7", "body8") * trvec2tform([0 0 h8/2]);
    c8 = collisionCylinder(r, h8, "Pose", t08);
    addCollision(robot.getBody("body8"), c8);
    addVisual(robot.getBody("body8"), "Cylinder", [r h8], t08);
    
    % --- New Block for Body9 (shifted from previous Body8) ---
    h9 = abs(dhparams(9,3));
    t09 = getTransform(robot, q, "body8", "body9") * trvec2tform([0 0 h9/2]);
    c9 = collisionCylinder(r, h9, "Pose", t09);
    addCollision(robot.getBody("body9"), c9);
    addVisual(robot.getBody("body9"), "Cylinder", [r h9], t09);
    
    % --- New Block for Body10 (shifted from previous Body9) ---
    h10 = abs(dhparams(10,3));
    t10 = getTransform(robot, q, "body9", "body10") * trvec2tform([0 0 0]);
    c10 = collisionCylinder(r, h10, "Pose", t10);
    addCollision(robot.getBody("body10"), c10);
    addVisual(robot.getBody("body10"), "Cylinder", [r h10], t10);
    
    % --- New Block for Body11 (shifted from previous Body10) ---
    % h11 = abs(dhparams(11,3));
    % t11 = getTransform(robot, q, "body10", "body11") * trvec2tform([0 0 -h11/2]);
    % c11 = collisionCylinder(r, h11, "Pose", t11);
    % addCollision(robot.getBody("body11"), c11);
    % addVisual(robot.getBody("body11"), "Cylinder", [r h11], t11);
    
    % --- New Block for Body12 (shifted from previous Body11) ---
    h12 = abs(dhparams(12,3));
    t12 = getTransform(robot, q, "body11", "body12") * trvec2tform([0 0 h12/2]);
    c12 = collisionCylinder(r, h12, "Pose", t12);
    addCollision(robot.getBody("body12"), c12);
    addVisual(robot.getBody("body12"), "Cylinder", [r h12], t12);
    
    % --- New Block for Body13 (shifted from previous Body12) ---
    h13 = abs(dhparams(13,3));
    t13 = getTransform(robot, q, "body12", "body13") * trvec2tform([0 0 h13/2]);
    c13 = collisionCylinder(r, h13, "Pose", t13);
    addCollision(robot.getBody("body13"), c13);
    addVisual(robot.getBody("body13"), "Cylinder", [r h13], t13);
    
    % --- New Block for Body14 (shifted from previous Body13) ---
    h14 = abs(dhparams(14,3));
    t14 = getTransform(robot, q, "body13", "body14") * trvec2tform([0 0 h14/2]);
    c14 = collisionCylinder(r, h14, "Pose", t14);
    addCollision(robot.getBody("body14"), c14);
    addVisual(robot.getBody("body14"), "Cylinder", [r h14], t14);
    
    % --- New Block for Body16 (shifted from previous Body15) ---
    h16 = abs(dhparams(16,3));
    t16 = getTransform(robot, q, "body15", "body16") * trvec2tform([0 0 h16+0.1]);
    c16 = collisionCylinder(0.02, h16, "Pose", t16);
    addCollision(robot.getBody("body16"), c16);
    addVisual(robot.getBody("body16"), "Cylinder", [0.02 h16], t16);
    
    % --- New Block for Body17 (shifted from previous Body16) ---
    h17 = abs(dhparams(17,3));
    t17 = getTransform(robot, q, "body16", "body17") * trvec2tform([0 0 h17/2]);
    c17 = collisionCylinder(0.02, h17, "Pose", t17);
    addCollision(robot.getBody("body17"), c17);
    addVisual(robot.getBody("body17"), "Cylinder", [0.02 h17], t17);


end
