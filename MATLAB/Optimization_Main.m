clear all;
clc; 
close all;

%Declare global variables to access during optimization
global bestCandidateGlobal bestCandidateObj designTrace reachTrace foldTrace j8Trace j10Trace
bestCandidateGlobal = [];
bestCandidateObj = inf;

designTrace = [];
reachTrace  = [];
foldTrace   = [];
j8Trace = [];
j10Trace = [];

% Create a DataQueue and specify a callback function that updates the global best candidate.
dq = parallel.pool.DataQueue;
afterEach(dq, @updateBestCandidate);
afterEach(dq, @recordMetrics);

%Disable annoying warnings
warning('off','robotics:robotmanip:joint:ResettingHomePosition');
warning('off', 'all');


% Specify DH parameters
dhparams_full = [
    % a       alpha    d     theta
       0     -pi/2    -0.306      0;
       0     -pi/2   10      0;
       0      pi/2   0.108     0;
       0     -pi/2   0.2     0;
       0      0     1.37     pi/2; %change this cylinder diameter to 0.061
       0.249    pi/2     0     0; % and this one
       0    -pi/2     -0.015      0
       0     -pi/2    0.509    0; %min 0.194 max 0.6
       0      pi/2    0.136     0; %min 0.81 max 0.15
       0     -pi/2    0.252    0; %min 0.14 max 0.6
       0      pi/2    0.116    0; %min 0.081 max 0.15
       0        0      0.424    0; % min 0.14 max 0.6
];

% Define joint types and home configuration
rev = "revolute";
pris = "prismatic";
fix = "fixed";
jointTypes = [fix, pris, rev, rev, fix, pris, fix, rev, rev, rev, rev, rev];
q_home = [0 0 0 0 0 0 0 0 0];
q_fold1 = [0 pi/2 pi/2 -0.9 0 0 0 pi 0];
q_fold2 = [0 pi/2 pi/2 -0.9 0 pi 0 pi 0];
q_test = [0 0 0 0 0 0 0 0 0];

% Create a rigidBodyTree robot model.
robot = createRobotCollisionModel(dhparams_full, jointTypes, q_home)

robot.getBody("body2").Joint.PositionLimits = [0, 10];
robot.getBody("body6").Joint.PositionLimits = [-1, 0];

% Define joint limits
jointLimits = [
    0, 10;     % Joint 2
    -pi, pi;   % Joint 3
    -pi, pi;   % Joint 4
    -1, 0;    % Joint 5
    -pi, pi;   % Joint 6
    -pi, pi;   % Joint 7
    -pi, pi;   % Joint 8
    -pi, pi;   % Joint 9
    -pi, pi;
];

%Define goal region
goalRegion = workspaceGoalRegion(robot.BodyNames{end});
goalRegion.ReferencePose = trvec2tform([0 0.466 -0.413]);
goalRegion.Bounds(1, :) = [-0.112 0.16];    % X Bounds
goalRegion.Bounds(2, :) = [0 1];    % Y Bounds
goalRegion.Bounds(3, :) = [-0.08 0.114];    % Z Bounds
goalRegion.Bounds(4, :) = [-0 0];  % Rotation about the X-axis
goalRegion.Bounds(5, :) = [-0 0];  % Rotation about the Y-axis
goalRegion.Bounds(6, :) = [-0 0];  % Rotation about the Z-axis

% setup visualisation
figure;
ax = axes;
show(robot, q_test, "Collisions", "on", 'Parent', ax);
title(ax, 'Optimized Robot - Home Configuration');
view(ax, 3);
axis(ax, [-3 3 -2 10 -5 1]);
hold(ax, 'on');
hold off;

% Wait for user input
disp('Press any key to start path planning...');
pause;

% Define goal poses
goalPoses = [
    -0.476, 2.015, -1.400, 0, 1, 0, -pi;  % Front BLM
    -0.476, 6.400, -1.200, 0, 1, 0, -pi;
    -1.469, 6.921, -1.400, 0, 1, 0, -pi;  % Back BLM
];

% Define obstacle environment
collisionCylinders = {
    collisionCylinder(0.457, 8.0)...   % Main Pipe
    collisionCylinder(0.044, 0.3)...   % Front BLM
    collisionCylinder(0.044, 0.3)...   % Back BLM
    collisionCylinder(0.350, 1.5)...   % First Obstacle
    collisionCylinder(0.350, 1.5)...   % Second Obstacle
};

collisionCylinders{1}.Pose = trvec2tform([-0.958, 4.260, -1.707]) * axang2tform([1, 0, 0, pi/2]);
collisionCylinders{2}.Pose = trvec2tform([-0.458, 2.015, -1.627]) * axang2tform([1, 0, 0, pi/2]);
collisionCylinders{3}.Pose = trvec2tform([-1.456, 6.921, -1.627]) * axang2tform([1, 0, 0, pi/2]);
collisionCylinders{4}.Pose = trvec2tform([-0.952, 5.400, -0.6]) * axang2tform([1, 0, 0, 0]);
collisionCylinders{5}.Pose = trvec2tform([-0.952, 6.900, -0.6]) * axang2tform([1, 0, 0, 0]);

% Define original variables 
originalVars = table();
originalVars.a6 = dhparams_full(6,1);
originalVars.d7 = dhparams_full(7,3);
originalVars.d8 = dhparams_full(8,3);
originalVars.d9 = dhparams_full(9,3);
originalVars.d10 = dhparams_full(10,3);
originalVars.d11 = dhparams_full(11,3);
originalVars.d12 = dhparams_full(12,3);


% Define optimization variables
optVars = [ optimizableVariable('a6', [0.1, 0.25], 'Type', 'real'), ...
            optimizableVariable('d7', [-0.2, 0.2], 'Type', 'real'), ...
            optimizableVariable('d8', [0.48, 0.8], 'Type', 'real'), ...
            optimizableVariable('d9', [0.081, 0.15], 'Type', 'real'), ...
            optimizableVariable('d10', [0.243, 0.8], 'Type', 'real'), ...
            optimizableVariable('d11', [0.081, 0.15], 'Type', 'real'), ...
            optimizableVariable('d12', [0.36, 0.8], 'Type', 'real')];

%optVars = [optimizableVariable('d10', [0.243, 0.8], 'Type', 'real')];

% objFcn = @(x) ObjectiveFcn_Collision(x, dhparams_full, jointTypes, goalPoses, q_home);
%objFcn = @(x) ObjectiveFcn_Fold(x, dhparams_full, jointTypes, goalRegion, q_home, dq);
objFcn = @(x) ObjectiveFcn_Full(x, dhparams_full, jointTypes,goalPoses, goalRegion, q_home, dq);

% Choose which parameters to use for convergence heat map
xvar = 'd12';
yvar = 'd10';


%% Run Bayesian Optimization     'UseParallel', true, ...
%% IF not starting then check objective function if debug is on!
results = bayesopt(objFcn, optVars, ...
    'MaxObjectiveEvaluations', 700, ...
    'UseParallel', true, ...
    'IsObjectiveDeterministic', true, ...
    'PlotFcn',{@plotObjective, @plotMinObjective,@(r,s) plotHeatmap(r,s,xvar,yvar), @plotMetrics}, ...
    'AcquisitionFunctionName', 'expected-improvement-plus', ...
    'Verbose', 2 ...
    );

%%Create new folder for this run
plotsDir = fullfile(pwd, 'plots');
if ~exist(plotsDir, 'dir')
    mkdir(plotsDir);
end

% Find existing run directories and determine next run number
d = dir(fullfile(plotsDir, 'run*'));
dirs = d([d.isdir]);  % keep only directories
runNums = [];
for k = 1:numel(dirs)
    name = dirs(k).name;
    % match names of the form 'runN'
    tok = regexp(name, '^run(\d+)$', 'tokens');
    if ~isempty(tok)
        runNums(end+1) = str2double(tok{1}{1});  %#ok<SAGROW>
    end
end
if isempty(runNums)
    nextRun = 1;
else
    nextRun = max(runNums) + 1;
end

% Make the new run folder
runFolder = fullfile(plotsDir, sprintf('run%d', nextRun));
mkdir(runFolder);

% Build full table of all samples and objectives
Tall = results.XTrace;                            % N×D table of a6…d12
Tall.objective = results.ObjectiveTrace;          % add the objective column

% Find indices of the 5 best objective values
[~, idxSort] = sort(Tall.objective, 'ascend');
top5Idx      = idxSort(1:min(5,height(Tall)));

% Extract top-5 rows
T5 = Tall(top5Idx, :);  

% Compute design_sum for each confgiuration
T5.design_sum = abs(T5.d8) + abs(T5.d10) + abs(T5.d12);

% Pull the recorded best‐roll angles from, DataQueue 
T5.r8  = j8Trace(top5Idx);
T5.r10 = j10Trace(top5Idx);

% Save the output file
writetable(T5, fullfile(runFolder,'top5.csv'));
save(fullfile(runFolder,'top5.mat'), 'T5');

%% plot dh parameters versus protruding volume
plotParamIsolation(results, dhparams_full, jointTypes, goalRegion, bestCandidateGlobal);


%% log-scale convergence plot
bestObj = cummin(results.ObjectiveTrace);

figure('Name','Objective vs. Iteration (log scale)','NumberTitle','off');
semilogy(1:numel(bestObj), bestObj,'LineWidth',0.5);
xlabel('BO Iteration');
ylabel('Best Objective Value (log scale)');
title('Convergence of BO (Log Y-Axis)');
grid on;

%% Extract best parameters
disp('Original design variables (for joints 6 and up):');
disp(originalVars);
bestVars = bestPoint(results);
disp('Best design variables:');
disp(bestVars);

% Update DH parameters using optimized design variables
dhparams_opt = dhparams_full;
dhparams_opt(6,1)  = bestVars.a6;
dhparams_opt(7,3)  = bestVars.d7;
dhparams_opt(8,3)  = bestVars.d8;
dhparams_opt(9,3)  = bestVars.d9;
dhparams_opt(10,3) = bestVars.d10;
dhparams_opt(11,3) = bestVars.d11;
dhparams_opt(12,3) = bestVars.d12;

disp('Updated DH Parameters:');
disp(dhparams_opt);

% Create the optimized robot model
[robot_opt, collisionData] = createRobotCollisionModel(dhparams_opt, jointTypes, q_home);

%check for errors in the model
%showdetails(robot_opt)

q_home = homeConfiguration(robot_opt);
q_fold = bestCandidateGlobal;

%specify the list of bodies that you ignore self collisions around.
%Standard is between child and parent and then add some specific ones.
adjbodynames = [robot_opt.Base.Name robot_opt.BodyNames];

skiplist = cell(robot_opt.NumBodies,2);
for i = 1:robot_opt.NumBodies
    skiplist(i,:) = {adjbodynames{i}, adjbodynames{i+1}};
end
%add the specific bodies to the skiplist
skiplist = [skiplist; {'body6','body8'}];

%% Shut down paralell pool
delete(gcp('nocreate'));

%% Folding test!

% setup visualisation
figure;
ax = axes;
show(robot_opt, bestCandidateGlobal, "Collisions", "on", 'Parent', ax);
title(ax, 'Optimized Robot - Home Configuration');
view(ax, 3);
axis(ax, [-3 3 -2 10 -3 1]);
hold(ax, 'on');
show(goalRegion);
hold off;

%% Goal Pose trajectory test!

%Update joint limits to allow movement along the rail
% Set joint limits 
jointLimits = [
    0, 10;     % Joint 2
    -pi, pi;   % Joint 3
    -pi, pi;   % Joint 4
    -1, 0;     % Joint 5
    -pi, pi;   % Joint 6
    -pi, pi;   % Joint 7
    -pi, pi;   % Joint 8
    -pi, pi;   % Joint 9
    -pi, pi;
];

robot_opt.getBody("body2").Joint.PositionLimits = [0,10];
% robot_opt.getBody("body3").Joint.PositionLimits = [-pi,pi];
% robot_opt.getBody("body4").Joint.PositionLimits = [-pi,pi];
robot_opt.getBody("body6").Joint.PositionLimits = [-1, 0];
% robot_opt.getBody("body6").Joint.HomePosition = 0;

q_home = [0 0 0 -0.9 0 0 0 0 0];%homeConfiguration(robot_opt);

% setup visualisation
figure;
ax = axes;
show(robot_opt, q_home, "Collisions", "on", 'Parent', ax);
title(ax, 'Optimized Robot - Home Configuration');
view(ax, 3);
axis(ax, [-3 3 -2 10 -3 1]);
hold(ax, 'on');

% Plot collision objects on the same axes
for i = 1:length(collisionCylinders)
    show(collisionCylinders{i}, 'Parent', ax);
end

% Plot goal poses on the same axes
for i = 1:size(goalPoses, 1)
    pos = goalPoses(i, 1:3);
    rot = goalPoses(i, 4:7);
    poseTF = trvec2tform(pos) * axang2tform(rot);
    plotTransforms(pos, tform2quat(poseTF), 'FrameSize', 0.1, 'Parent', ax);
    text(ax, pos(1), pos(2), pos(3), sprintf('Pose %d', i), 'FontSize', 10, 'Color', 'r');
end

%% Save all current figures into this run folder, both .png and interactive .fig
figs = findall(groot, 'Type', 'figure');
for f = 1:numel(figs)
    h = figs(f);
    name = h.Name;
    if isempty(name)
        name = sprintf('fig%d', f);
    end

    % PNG snapshot
    %saveas(h, fullfile(runFolder, [name '.png']));

    % Interactive MATLAB figure
    savefig(h, fullfile(runFolder, [name '.fig']));
end

%Set up ik solver
gik = generalizedInverseKinematics('RigidBodyTree', robot_opt, ...
                                   'ConstraintInputs', {'pose', 'joint'});

jointBounds = constraintJointBounds(robot_opt);
jointBounds.Bounds = jointLimits;

planner = manipulatorRRT(robot_opt,collisionCylinders);
planner.SkippedSelfCollisions = skiplist;
planner.EnableConnectHeuristic = false;
planner.MaxConnectionDistance = 0.1;
planner.ValidationDistance = 0.005;
planner.MaxIterations = 10000;
rng(0)

videoFile = fullfile(runFolder, "trajectory.avi");
v = VideoWriter(videoFile);
v.FrameRate = 20;
q_guess = q_home;  % Start from the home configuration

%% Wait for user input
disp('Ready to record path planning! press any key to start...');
pause;

open(v);  

for i = 1:size(goalPoses, 1)
    % Define the pose constraint for the current goal pose.
    poseTgt = constraintPoseTarget(robot_opt.BodyNames{end});
    poseTgt.TargetTransform = trvec2tform(goalPoses(i, 1:3)) * axang2tform(goalPoses(i, 4:7));

    % Solve the constrained IK problem using the current guess.
    [q_sol, solInfo] = gik(q_guess, poseTgt, jointBounds);

    % check for issues with the goal pose
    if solInfo.Status ~= "success"
        warning('IK failed for goal %d. Skipping trajectory planning for this goal.', i);
        q_guess = q_sol;  % update guess and continue
        continue;
    end

    % Pre-check the goal configuration for collision.
    if checkCollision(robot_opt, q_sol, collisionCylinders, 'SkippedSelfCollisions', skiplist)
        warning('Goal configuration for pose %d is in collision. Skipping trajectory planning for this goal.', i);
        q_guess = q_sol;
        continue;
    end

    % Try to plan a collision-free trajectory.
    try
        collisionFreePath = planner.plan(q_guess, q_sol);
    catch ME
        warning('RRT planning failed for goal %d: %s', i, ME.message);
        collisionFreePath = [];
    end

    if isempty(collisionFreePath)
        warning('No collision-free path found for pose %d.', i);
    else      
        % Animate the collision-free trajectory and check for collisions with obstacles.
        for j = 1:size(collisionFreePath, 1)
            q_traj = collisionFreePath(j,:);
            if checkCollision(robot_opt, q_traj, collisionCylinders, 'SkippedSelfCollisions', skiplist)
                warning('Collision detected along trajectory at configuration %d for pose %d.', j, i);
            end
            show(robot_opt, q_traj, 'collisions', 'on', 'Parent', ax, 'PreservePlot', false);
            title(ax, sprintf('Moving to Goal Pose %d', i));
            drawnow;
            frame = getframe(gcf);
            writeVideo(v, frame);
            %pause(0.05);
        end
    end

    % Update the starting configuration for the next pose.
    q_guess = q_sol;

    %pause(2);
    disp('Press Enter to move to the next pose...');
    pause;
    disp('Moving...')
end

 close(v);

 disp(bestCandidateGlobal);
 disp(bestCandidateObj);

close all;
clear all;

%% Function to extract the best configuration based on objective value
function updateBestCandidate(candidateStruct)
    global bestCandidateGlobal bestCandidateObj
    if candidateStruct.value < bestCandidateObj
        bestCandidateObj = candidateStruct.value;
        bestCandidateGlobal = candidateStruct.q_candidate;
        fprintf('Best Objective Updated: %.4f\n', bestCandidateObj);
    end   
end

function recordMetrics(s)
    % s is the struct your objective sends
    global designTrace reachTrace foldTrace j8Trace j10Trace

    designTrace(end+1,1) = s.designSum;
    reachTrace (end+1,1) = s.reachability;
    foldTrace  (end+1,1) = s.foldability;
    j8Trace    (end+1,1) = s.j8;
    j10Trace   (end+1,1) = s.j10;
end
