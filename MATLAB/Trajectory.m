%Load robot model, set joint angles for home position

clear all;
clc; 
%close all;

% Specify Dh parameters
dhparams = [
    %a(t_x)   alpha(r_x)   d(t_z)    theta(r_z) 
    0   -pi/2  0  0;
    0   -pi/2  10 0;
    0    pi/2  0.1 0;
    0    -pi/2  0.1 0;
    0    pi/2  1.5 0;
    0   -pi/2  0.1 0;
    0   pi/2   1   0;
    ];

% Define the joint types

rev = "revolute";
pris = "prismatic";
fix = "fixed";
jointTypes = [rev, pris, rev, rev, pris, rev, rev];
q_home = [0, 0, 0, 0 0 0 0];

% Call function to create rigid body tree model

robot = createRobotModel_Generalized(dhparams, jointTypes, q_home)
showdetails(robot);
robot.getBody("body2").Joint.PositionLimits = [0, 10];
robot.getBody("body1").Joint.PositionLimits = [0, 0];
robot.getBody("body5").Joint.PositionLimits = [0, 1.4];


% Check for collisions in home configuration
% 
% [isColliding, sepDist, witnessPts] = checkCollision(robot, q_home);
% 
% if isColliding
%     disp('The robot is in self-collision in the home configuration.');
% else
%     disp('No self-collision detected in the home configuration.');
% end

%Set joint limits and show arm in home position
config = homeConfiguration(robot);
eeName = robot.BodyNames{end};

%gui = interactiveRigidBodyTree(robot,"Configuration",config)
figure;
show(robot,q_home, Visuals="off", Collisions="on");
axis equal;          
xlim([-10, 10]);   
ylim([-10, 10]);      
zlim([-10, 10]);   
view(3);   
hold on

%Generate environment
collisionCylinders = {
    collisionCylinder(0.457, 8.0),  % Main Pipe
    collisionCylinder(0.044, 0.3),  % Front BLM
    collisionCylinder(0.044, 0.3)  % Back BLM
    %collisionCylinder(0.350, 1.5),  % First Obstacle
    %collisionCylinder(0.350, 1.5)   % Second Obstacle
};

cylinderPoses = [
    -0.958, 4.260, -1.707, 1, 0, 0, pi/2; 
    -0.458, 2.015, -1.627, 1, 0, 0, pi/2;
    -1.456, 6.921, -1.627, 1, 0, 0, pi/2; 
    %-0.952, 5.400, -0.986, 1, 0, 0, 0;
    %-0.952, 6.900, -0.986, 1, 0, 0, 0
];

for i = 1:length(collisionCylinders)
    P = trvec2tform(cylinderPoses(i, 1:3));
    R = axang2tform(cylinderPoses(i, 4:7));
    collisionCylinders{i}.Pose = P * R;
    show(collisionCylinders{i});
end

%Define goal poses
goalPoses = [
          -0.476, 2.015, -1.400, 1, 0, 0, -pi/2;  % Front BLM
          -0.476, 6.200, -1.400, 1, 0, 0, -pi/2;  % Front Obstacles
          -0.900, 6.300, -1.000, 1, 0, 0, -pi/2;  % Between Obstacles
          -1.469, 6.400, -1.400, 1, 0, 0, -pi/2;  % Back Obstacles
          -1.469, 6.921, -1.400, 1, 0, 0, -pi/2;  % Back BLM
            ];

disp('Plotting all target poses...');
for i = 1:size(goalPoses, 1)
    pos = goalPoses(i, 1:3);
    rot = goalPoses(i, 4:7);
    poseTF = trvec2tform(pos) * axang2tform(rot);
    
    plotTransforms(pos, tform2quat(poseTF), "FrameSize", 0.1);
    text(pos(1), pos(2), pos(3), sprintf('Pose %d', i), 'FontSize', 10, 'Color', 'r');
end

%Create RRT trajectory planner

planner = manipulatorRRT(robot,collisionCylinders);
planner.SkippedSelfCollisions = 'parent';
planner.EnableConnectHeuristic = true
planner.MaxConnectionDistance = 0.2;
planner.ValidationDistance = 0.005;
planner.MaxIterations = 100;
rng(0)

%Create IK solver

ik = inverseKinematics('RigidBodyTree', robot);
ikWeights = [1 1 1 1 1 1]; % [orientation weights, then position weights]
qInitial = config;

numGoals = size(goalPoses, 1);
qSolutions = zeros(numel(config), numGoals);
errorVectors = zeros(6, numGoals);  % to store 6-element error vectors

for i = 1:numGoals
    % Wait for user input before proceeding to the next pose.
    disp('Press Enter to move to the next pose...');
    pause;
    
    % Extract desired pose (translation and axis-angle rotation)
    pos = goalPoses(i, 1:3);
    axang = goalPoses(i, 4:7);
    targetTform = trvec2tform(pos) * axang2tform(axang);
    
    % Compute IK solution from current qInitial.
    [qSol, solInfo] = ik(eeName, targetTform, ikWeights, qInitial);
    qSolutions(:, i) = qSol(:);
    
    % Plan a collision-free path using the RRT planner:
    [pthObj, solnInfo] = plan(planner, qInitial, qSol);
    if solnInfo.IsPathFound
        % Extract the path states from the planner.
        qPath = pthObj;
        
        % Optionally, interpolate the discrete path for smooth animation:
        numInterpSteps = 200;
        q_traj = trapveltraj(qPath', numInterpSteps);
        
        % Animate the trajectory:
        for k = 1:size(q_traj,2)
            qCurrent = q_traj(:, k);
            show(robot, qCurrent', 'PreservePlot', false, 'Visuals','off','Collisions','on');
            title(sprintf('Moving from Pose %d to Pose %d, step %d/%d', i, i, k, numInterpSteps));
            drawnow;
            pause(0.02);  % adjust for desired animation speed
        end
    else
        disp('Collision-free path not found. Consider adjusting planner parameters.');
    end
    
    % After finishing the trajectory, compute the error vector at final pose.
    T_actual = getTransform(robot, qSol, eeName);
    posDesired = tform2trvec(targetTform);
    posActual  = tform2trvec(T_actual);
    posErrorVector = posDesired - posActual;  % [ex, ey, ez]
    
    % Orientation error:
    R_desired = tform2rotm(targetTform);
    R_actual  = tform2rotm(T_actual);
    R_err = R_actual' * R_desired;  % relative rotation: R_actual * R_err = R_desired
    axang_err = tform2axang([R_err, zeros(3,1); 0 0 0 1]);
    orientErrorVector = axang_err(1:3) * axang_err(4);  % [erx, ery, erz]
    
    errorVector = [posErrorVector, orientErrorVector];
    errorVectors(:, i) = errorVector;
    
    fprintf('Pose %d error vector: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', ...
        i, errorVector(1), errorVector(2), errorVector(3), ...
        errorVector(4), errorVector(5), errorVector(6));
    
    % Update initial configuration for the next pose.
    qInitial = qSol;
end
