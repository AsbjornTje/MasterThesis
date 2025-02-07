clear all;
clc;
%close all;

q = [0 0 0 0 0 0 0 0];

% Specify Dh parameters
dhparams = [
    %a(t_x)   alpha(r_x)   d(t_z)    theta(r_z) 
    0.000     -pi/2        -0.0745   0.000;      % 1 shoulder yaw   
    0.000     -pi/2        -0.183    0.000;      % 2 shoulder pitch
    0.000      0.000        0.500   -pi/2;       % 3 fixed prismatic link
    0.000      0.000        1.370    0.000;      % 4 shoulder prismatic joint
    0.150      0.000        0.000    0.000;      % 5 fixed prismatic link offset
    0.000      0.000        0.100    0.000;      % 6 first yaw joint new arm
    0.000      pi/2         0.300    0.000;      % 7 l1
    0.000      0.000        0.100    0.000;      % 8 first pitch joint
    0.000     -pi/2         0.000    0.000;      % 9 l2
    0.000     0.000         0.100    0.000;      % 10 second yaw joint
    0.000     pi/2          0.300    0.000;      % 11 l3
    0.000     0.000         0.100    0.000;      % 12 second pitch joint
    0.000     -pi/2         0.000    0.000;      % 13 l4
    0.000     0.000         0.100    0.000;      % 14 third yaw joint
    0.000     0.000         0.200    0.000;      % 15 End effector
    ];

% Specify joint types
rev = "revolute";
pris = "prismatic";
fix = "fixed";
jointTypes = [rev, rev, fix ,pris, fix, rev,fix, rev, fix, rev, fix, rev, fix, rev, fix, rev, fix];

% Define size of the dh table and number of bodies and joints
numJoints = size(dhparams, 1);
bodies = cell(numJoints,1);
joints = cell(numJoints,1);

% Build rigidBodyTree
robot = rigidBodyTree("DataFormat","row");

for i = 1:numJoints
    bodies{i} = rigidBody(['body' num2str(i)]);
    
    % Ensure jointTypes is properly defined
    if length(jointTypes) < numJoints
        error("jointTypes array must have the same length as dhparams.");
    end
    
    % Type of joint to be added
    joints{i} = rigidBodyJoint(['jnt' num2str(i)], jointTypes(i));
    
    setFixedTransform(joints{i}, dhparams(i,:), "dh");
    bodies{i}.Joint = joints{i};
    
    if i == 1 % Add first body to base
        addBody(robot, bodies{i}, "base");
    else % Add current body to previous body by name
        addBody(robot, bodies{i}, bodies{i-1}.Name);
    end
end

% Add Collision objects to the bodies
% Specify link radius
r = 0.07;

h1 = abs(dhparams(1,3));
t01 = getTransform(robot,q,"base","body1") * trvec2tform([0 0 -h1/2]);
c1 = collisionCylinder(r, h1, "Pose",t01);
addCollision(robot.getBody("body1"), c1);
addVisual(robot.getBody("body1"),"Cylinder",[r h1],t01);

h2 = abs(dhparams(2,3));
t02 = getTransform(robot,q,"body1","body2") * trvec2tform([0 0 -h2/2]);
c2 = collisionCylinder(r, h2,"Pose",t02);
addCollision(robot.getBody("body2"),c2);
addVisual(robot.getBody("body2"),"Cylinder",[r h2],t02);

h3 = abs(dhparams(3,3))*2;
t03 = getTransform(robot,q,"body2","body3") * trvec2tform([0 0 h3/2]);
c3 = collisionCylinder(0.065, h3,"Pose",t03);
addCollision(robot.getBody("body3"),c3);
addVisual(robot.getBody("body3"),"Cylinder",[0.065 h3],t03);

h5 = abs(dhparams(5,1));
t05 = getTransform(robot,q,"body4","body5") * trvec2tform([h5 0 0]);
c5 = collisionCylinder(r, h5,"Pose",t05);
addCollision(robot.getBody("body5"),c5);
addVisual(robot.getBody("body5"),"Cylinder",[r h5],t05);

h6 = abs(dhparams(6,3));
t06 = getTransform(robot,q,"body5","body6") * trvec2tform([0 0 h6]);
c6 = collisionCylinder(r, h6,"Pose",t06);
addCollision(robot.getBody("body6"),c6);
addVisual(robot.getBody("body6"),"Cylinder",[r h6],t06);

h7 = abs(dhparams(7,3));
t07 = getTransform(robot,q,"body6","body7") * trvec2tform([0 0 h7/2]);
c7 = collisionCylinder(r, h7,"Pose",t07);
addCollision(robot.getBody("body7"),c7);
addVisual(robot.getBody("body7"),"Cylinder",[r h7],t07);

h8 = abs(dhparams(8,3));
t08 = getTransform(robot,q,"body7","body8") * trvec2tform([0 0 0]);
c8 = collisionCylinder(r, h8,"Pose",t08);
addCollision(robot.getBody("body8"),c8);
addVisual(robot.getBody("body8"),"Cylinder",[r h8],t08);

h9 = abs(dhparams(8,3));
t09 = getTransform(robot,q,"body8","body9") * trvec2tform([0 0 -h9/2]);
c9 = collisionCylinder(r, h9,"Pose",t09);
addCollision(robot.getBody("body9"),c9);
addVisual(robot.getBody("body9"),"Cylinder",[r h9],t09);

h10 = abs(dhparams(10,3));
t10 = getTransform(robot,q,"body9","body10") * trvec2tform([0 0 h10/2]);
c10 = collisionCylinder(r, h10,"Pose",t10);
addCollision(robot.getBody("body10"),c10);
addVisual(robot.getBody("body10"),"Cylinder",[r h10],t10);

h11 = abs(dhparams(11,3));
t11 = getTransform(robot,q,"body10","body11") * trvec2tform([0 0 h11/2]);
c11 = collisionCylinder(r, h11,"Pose",t11);
addCollision(robot.getBody("body11"),c11);
addVisual(robot.getBody("body11"),"Cylinder",[r h11],t11);

h12 = abs(dhparams(12,3));
t12 = getTransform(robot,q,"body11","body12") * trvec2tform([0 0 h12/2]);
c12 = collisionCylinder(r, h12,"Pose",t12);
addCollision(robot.getBody("body12"),c12);
addVisual(robot.getBody("body12"),"Cylinder",[r h12],t12);

h14 = abs(dhparams(14,3));
t14 = getTransform(robot,q,"body13","body14") * trvec2tform([0 0 h14/2]);
c14 = collisionCylinder(r, h14,"Pose",t14);
addCollision(robot.getBody("body14"),c14);
addVisual(robot.getBody("body14"),"Cylinder",[r h14],t14);

h15 = abs(dhparams(15,3));
t15 = getTransform(robot,q,"body14","body15") * trvec2tform([0 0 h15/2]);
c15 = collisionCylinder(r, h15,"Pose",t15);
addCollision(robot.getBody("body15"),c15);
addVisual(robot.getBody("body15"),"Cylinder",[r h15],t15);

% Set joint limits
robot.getBody('body4').Joint.PositionLimits = [-0.2, 0.5];

%Perform forward kinematics


% Display the details of the constructed manipulator for verification
disp('Constructed a rigidBodyTree from DH parameters:');
showdetails(robot);

% Display the manipulator in 3d space
%gui = interactiveRigidBodyTree(robot,"Configuration",q)
figure;
show(robot,q, "visuals", "off", "Collisions","on");

% disp("Press any key to close")
% pause('on');
%     pause;
%close All;

