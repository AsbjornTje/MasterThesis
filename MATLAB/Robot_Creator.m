clear all;
clc;
%close all;

% Specify Dh parameters
dhparams = [
    %a(length along x)     alpha(rotation around x)    d(length along Z)      theta(rotation around z) 
     0       -pi    0.4    0;         
     0.2     0.5*pi   0.3    0;
     0        -0.5*pi   1     0;
     0.1          0       0.5    0
];

% Specify joint types
rev = "revolute";
pris = "prismatic";
jointTypes = [rev, rev, pris, rev];

% Specify link radius
r = 0.07;

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

     if jointTypes(i) == "revolute"
        % Cylinder for revolute joints
        if dhparams(i,3) ~= 0  % If 'd' (length along z) is nonzero, use it for height
            h = abs(dhparams(i,3)); 
        elseif dhparams(i,1) ~= 0  % Otherwise, use 'a' (length along x)
            h = abs(dhparams(i,1));
        else
            h = 0.1; % Default small cylinder
        end
        collisionObj = collisionCylinder(r, h);
        
        % Positioning: Shift cylinder along the correct axis
        if dhparams(i,3) ~= 0
            T = trvec2tform([0, 0, h/2]); % Shift along z
        elseif dhparams(i,1) ~= 0
            T = trvec2tform([h/2, 0, 0]); % Shift along x
        else
            T = eye(4);
        end
    
    elseif jointTypes(i) == "prismatic"
        % Box for prismatic joints (to represent sliding)
        boxLength = max(abs(dhparams(i,3)), 0.3); % Ensure a reasonable box size
        boxWidth = 0.1; % Fixed width
        boxHeight = 0.1; % Fixed height
        collisionObj = collisionBox(boxLength, boxWidth, boxHeight);
        
        % Positioning: Shift box along the Z-axis to match prismatic motion
        T = trvec2tform([0, 0, boxLength / 2]);
    end
    
    % Add collision object with transformation
    addCollision(robot.Bodies{i}, collisionObj, T);
end

%     % Adjust the collision object size dynamically
%     % Use 'a' (link length along x) or 'd' (link length along z) to determine height
%     if dhparams(i,3) ~= 0  % If 'd' parameter (prismatic displacement) is nonzero, use it for height
%         h = abs(dhparams(i,3)); 
%     elseif dhparams(i,1) ~= 0  % Otherwise, use 'a' parameter (revolute link length)
%         h = abs(dhparams(i,1));
%     else
%         h = 0.1; % Default small cylinder if no clear link length is defined
%     end
% 
%     % Create collision object
%     collisionObj = collisionCylinder(r, h);
% 
%     % Transform collision object so it aligns with the link
%     % Shift along x if using 'a', shift along z if using 'd'
%     if dhparams(i,3) ~= 0
%         T = trvec2tform([0, 0, h/2]); % Shift along z
%     elseif dhparams(i,1) ~= 0
%         T = trvec2tform([h/2, 0, 0]); % Shift along x
%     else
%         T = eye(4); % No shift needed
%     end
% 
%     % Add the collision object with transformation
%     addCollision(robot.Bodies{i}, collisionObj, T);
% end

% Display the details of the constructed manipulator for verification
disp('Constructed a rigidBodyTree from DH parameters:');
showdetails(robot);

% Display the manipulator in 3d space
figure(Name="Manipulator plot");
show(robot,Collisions="on");

disp("Press any key to close")
pause('on');
    pause;
close All;

