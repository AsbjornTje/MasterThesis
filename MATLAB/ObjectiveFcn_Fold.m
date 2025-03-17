function objective = ObjectiveFcn_Fold(x, dhparams_full, jointTypes, goalRegion, q_home, dq)
    
    %Disable annoying warning
    warning('off','robotics:robotmanip:joint:ResettingHomePosition');

    % Update DH parameters using optimized variables
    dhparams_opt = dhparams_full;
    dhparams_opt(6,1)  = x.a6;
    dhparams_opt(7,3)  = x.d7;
    dhparams_opt(8,3)  = x.d8;
    dhparams_opt(9,3)  = x.d9;
    dhparams_opt(10,3)  = x.d10;
    dhparams_opt(11,3) = x.d11;
    dhparams_opt(12,3) = x.d12;
    
    % Accumulate design cost:
    sum_design = abs(x.a6) + abs(x.d7) + abs(x.d8) + abs(x.d9) + abs(x.d10) + abs(x.d11) + abs(x.d12);
    sum_volume = inf; %This guarantees the first candidate is better than starting value
    bestCandidateLocal = [];  % temporary best candidate for evaluation

    % Weights for the different components.
    penaltyWeight = 10;
    designWeight  = 1;
    volumeWeight = 7;

    % Define a penalty values for collision or planning failure.
    collisionPenaltyValue = 1000; 
        
    % Initialize accumulators for penalties.
    volumeCandidate = 0;
    
    % Create the robot model using updated DH parameters.
    [robot, collisionData]= createRobotCollisionModel(dhparams_opt, jointTypes, q_home);
    %disp(dhparams_opt)

    %specify the list of bodies that you ignore self collisions around.
    %Standard is between child and parent and then add some specific ones.
    adjbodynames = [robot.Base.Name robot.BodyNames];
    skiplist = cell(robot.NumBodies,2);
    for i = 1:robot.NumBodies
        skiplist(i,:) = {adjbodynames{i}, adjbodynames{i+1}};
    end
    %add the specific bodies to the skiplist
    skiplist = [skiplist; {'body6','body8'}];
    
    %Set joint limits and joint home positions for the non folding parts
    robot.getBody("body2").Joint.PositionLimits = [0,0];
    robot.getBody("body3").Joint.PositionLimits = [pi/2,pi/2];
    robot.getBody("body3").Joint.HomePosition = pi/2;
    robot.getBody("body4").Joint.PositionLimits = [pi/2,pi/2];
    robot.getBody("body4").Joint.HomePosition = pi/2;
    robot.getBody("body6").Joint.PositionLimits = [-0.7, -0.7];
    robot.getBody("body6").Joint.HomePosition = -0.7;

    q_home = homeConfiguration(robot);
    q_fold1 = [0 pi/2 pi/2 -0.7 0 0 0 pi 0];
    q_fold2 = [0 pi/2 pi/2 -0.7 0 pi 0 pi 0];
  
    %Initialize path planner
    planner = manipulatorRRT(robot,{});
    planner.SkippedSelfCollisions = skiplist;
    planner.EnableConnectHeuristic = false;
    planner.MaxConnectionDistance = 0.5;
    planner.ValidationDistance = 0.005;
    planner.MaxIterations = 10000;
    rng(0);

    if x.d8 < 0.4
        baseCandidate = q_fold1;
    else
        baseCandidate = q_fold2;
    end
    disp(x.d8)
    disp(baseCandidate)
    
    %Attempt to reach the folded configuration without self collisions
    try
        rrtPath = planner.plan(q_home, baseCandidate);
    catch ME
        warning('RRT planning failed: %s', ME.identifier, '%s', ME.message);
        rrtPath = [];
    end
        
    if isempty(rrtPath)
        volumeCandidate = volumeCandidate + collisionPenaltyValue;
    else
        % Optionally, check each configuration along the path for collisions.
        for j = 1:size(rrtPath, 1)
            q_path = rrtPath(j, :);
            if checkCollision(robot, q_path ,'SkippedSelfCollisions', skiplist)
                volumeCandidate = volumeCandidate + collisionPenaltyValue;
                break;
            end
        end
    end 

    % Assume the roll joints to be varied are for body 8 and 10 .
    rollJointIndex8 = 5;
    rollJointIndex10 = 7;

    % Define full 360° range in increments.
    rollStep = deg2rad(30);
    rollRange = -pi:rollStep:pi;  % 360° sweep.

    % Loop over candidate roll angles.
    for r8 = rollRange
        for r10 = rollRange
            q_candidate = baseCandidate;
            q_candidate(rollJointIndex8) = r8;
            q_candidate(rollJointIndex10) = r10;

            % Evaluate extruding volume penalty. 
            %% NB!!!! 
            %% USE FALSE WHEN NOT TESTING TO AVOID GETTING STUCK IN VIZUALIZATION LOOP!!!!
            [volX, volY, volZ] = Protruding_Volume(robot, collisionData, q_candidate, goalRegion, 1000, false);
            volumeCandidate = volX + volY + volZ;

            if checkCollision(robot, q_candidate, 'SkippedSelfCollisions', skiplist)
                volumeCandidate = volumeCandidate + collisionPenaltyValue;
            end
            
            if volumeCandidate < sum_volume
                sum_volume = volumeCandidate;
                bestCandidateLocal = q_candidate;
            end
        end
        disp("volumeCandidate value:")
        disp(volumeCandidate);
    end

    disp(bestCandidateLocal)
    % Send candidate info to the DataQueue.
    send(dq, struct('q_candidate', bestCandidateLocal, 'value', sum_volume));
    
    % Final objective combines design cost and penalties.
    objective = penaltyWeight * volumeCandidate + designWeight * sum_design + volumeWeight * sum_volume;
    
    fprintf('Objective: %.4f, Total Penalty: %.4f, Design Sum: %.4f\n, Volume Sum: %.4f', ...
            objective, penaltyWeight * volumeCandidate, sum_design, sum_volume);
end