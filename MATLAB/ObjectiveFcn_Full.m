function objective = ObjectiveFcn_Full(x, dhparams_full, jointTypes, goalPoses, goalRegion, q_home, dq)
    
    %Disable annoying warnings
    warning('off','robotics:robotmanip:joint:ResettingHomePosition');
    warning('off', 'all');

    disp(' ');
    global bestCandidateGlobal bestCandidateObj dq

    % Update DH parameters using optimized variables
    dhparams_opt = dhparams_full;
    dhparams_opt(6,1)  = x.a6;
    dhparams_opt(7,3)  = x.d7;
    dhparams_opt(8,3)  = x.d8;
    dhparams_opt(9,3)  = x.d9;
    dhparams_opt(10,3) = x.d10;
    dhparams_opt(11,3) = x.d11;
    dhparams_opt(12,3) = x.d12;
    
    % Accumulate design cost:
    sum_design = abs(x.d8) + abs(x.d10) + abs(x.d12);
    %sum_design = abs(x.a6) + abs(x.d7) + abs(x.d8) + abs(x.d9) + abs(x.d10) + abs(x.d11) + abs(x.d12);
    %sum_design = abs(x.d10);
    sum_volume = inf; %This guarantees the first candidate is better than starting value
    bestCandidateLocal = [];  % temporary best candidate for evaluation
    bestRollCollisionPenalty = inf;
    bestJ8 = 0;
    bestJ10 = 0;

    % Define a penalty values for collision or planning failure.
    totalPoseError = 0;
    total_reachabilityPenalty   = 0;
    collisionPenaltyValue = 40; 
        
    % Initialize accumulators for penalties.
    volumeCandidate = 0;
    foldingMotion = 0;
    
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
    
    %%fOLDING PART

    % 
    q_home = homeConfiguration(robot);
    q_fold1 = [0 pi/2 pi/2 -0.7 0 0 0 pi 0];
    q_fold2 = [0 pi/2 pi/2 -0.7 0 pi 0 pi 0];
  
    %Initialize path planner
    planner_fold = manipulatorRRT(robot,{});
    planner_fold.SkippedSelfCollisions = skiplist;
    planner_fold.EnableConnectHeuristic = false;
    planner_fold.MaxConnectionDistance = 0.5;
    planner_fold.ValidationDistance = 0.005;
    planner_fold.MaxIterations = 10000;
    rng(0);

    % if x.d8 < 0.4
    %     baseCandidate = q_fold1;
    % else
    %     baseCandidate = q_fold2;
    % end
    % disp(x.d8)
    baseCandidate = q_fold2;
    %disp(baseCandidate)
    
    %Attempt to reach the folded configuration without self collisions
    try
        rrtPath = planner_fold.plan(q_home, baseCandidate);
    catch ME
        warning('RRT planning failed: %s', ME.identifier, '%s', ME.message);
        rrtPath = [];
    end
        
    if isempty(rrtPath)
        foldingMotion = foldingMotion + collisionPenaltyValue;
    else
        % Optionally, check each configuration along the path for collisions.
        for j = 1:size(rrtPath, 1)
            q_path = rrtPath(j, :);
            if checkCollision(robot, q_path ,'SkippedSelfCollisions', skiplist)
                foldingMotion = foldingMotion + collisionPenaltyValue;
                break;
            end
        end
    end 
    disp('folding motion penalty: ')
    disp(foldingMotion);

    % Assume the roll joints to be varied are for body 8 and 10 .
    rollJointIndex8 = 5;
    rollJointIndex10 = 7;

    % Define full 360° range in increments.
    rollStep = deg2rad(3);
    rollRange = -pi:rollStep:pi;  % 360° sweep.
    
    %% TEST
    % Initialize “best of each class”
    bestFreeVol     = inf;
    bestFreeCand    = [];
    bestCollideVol  = inf;
    bestCollideCand = [];
    penVal          = collisionPenaltyValue;

    % Sweep all roll angles
    for r8 = rollRange
      for r10 = rollRange
        q_candidate = baseCandidate;
        q_candidate(rollJointIndex8)  = r8;
        q_candidate(rollJointIndex10) = r10;
    
        % compute protrusion volume
        [volX,volY,volZ]   = Protruding_Volume(robot, collisionData, q_candidate, goalRegion, 5000, false);
        volumeCandidate    = 1000*(volX+volY+volZ);
    
        % check collision
        isColliding = checkCollision(robot, q_candidate, 'SkippedSelfCollisions', skiplist);
    
        if ~isColliding
          % collision‐free branch
          if volumeCandidate < bestFreeVol
            bestFreeVol  = volumeCandidate;
            bestFreeCand = q_candidate;
            bestJ8 = r8;
            bestJ10 = r10;
          end
        else
          % colliding branch
          if volumeCandidate < bestCollideVol
            bestCollideVol  = volumeCandidate;
            bestCollideCand = q_candidate;
            bestJ8 = r8;
            bestJ10 = r10;
          end
        end
      end
    end
    
    % Decide final “best” and penalty
    if ~isempty(bestFreeCand)
      % we found at least one collision-free config
      sum_volume               = bestFreeVol;
      bestCandidateLocal       = bestFreeCand;
      bestminRollCollisionPenalty = 0;
    else
      % all options collided: pick the best of them
      sum_volume               = bestCollideVol;
      bestCandidateLocal       = bestCollideCand;
      bestminRollCollisionPenalty = penVal;
    end

    disp('bestminRollCollisionPenalty:');
    disp(bestminRollCollisionPenalty);
    fprintf('sum_volume: %.4f', sum_volume);
    
    threshold = 2.2;    % your volume limit
    penaltyWeight = 10;     % how severe the penalty per unit over the limit
    
    % compute how far you are above the threshold
    excess = max(0, sum_volume - threshold);
    
    % add a **linear** penalty proportional to that excess
    sum_volume = sum_volume + penaltyWeight * excess;
    
    % optional: show if you’re over
    if excess > 0
        fprintf('  exceeded threshold by %.3f → +%.3f penalty\n', excess, penaltyWeight*excess);
    end  
   
    %% BLM MEASUREMENT PART

    % Define obstacle environment (collision cylinders)
    collisionCylinders = {
    collisionCylinder(0.457, 8.0)   % Main Pipe
    collisionCylinder(0.044, 0.3)   % Front BLM
    collisionCylinder(0.044, 0.3)   % Back BLM
    collisionCylinder(0.350, 1.5)   % First Obstacle
    collisionCylinder(0.350, 1.5)   % Second Obstacle
    };

    % Set their poses
    cylinderPoses = [
        -0.958, 4.260, -1.707, 1, 0, 0, pi/2; 
        -0.458, 2.015, -1.627, 1, 0, 0, pi/2;
        -1.456, 6.921, -1.627, 1, 0, 0, pi/2; 
        -0.952, 5.400, -0.6,    1, 0, 0, 0;
        -0.952, 6.900, -0.6,    1, 0, 0, 0
    ];

    for i = 1:length(collisionCylinders)
        P = trvec2tform(cylinderPoses(i,1:3));
        R = axang2tform(cylinderPoses(i,4:7));
        collisionCylinders{i}.Pose = P * R;
    end

    %set new joint limits to allow motion along the rail

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

    robot.getBody("body2").Joint.PositionLimits = [0, 10];
    % robot.getBody("body3").Joint.PositionLimits = [-pi,pi];
    % robot.getBody("body4").Joint.PositionLimits = [-pi,pi];
    robot.getBody("body6").Joint.PositionLimits = [-1, 0];

    % Initialize the constrained IK solver and a motion planner for collision checking.
    gik = generalizedInverseKinematics('RigidBodyTree', robot, 'ConstraintInputs', {'pose', 'joint'});
    jointBounds = constraintJointBounds(robot);
    jointBounds.Bounds = jointLimits;

    planner = manipulatorRRT(robot, collisionCylinders);
    planner.SkippedSelfCollisions = skiplist;
    planner.EnableConnectHeuristic = false;
    planner.MaxConnectionDistance = 0.1;
    planner.ValidationDistance = 0.005;
    planner.MaxIterations = 10000;
    rng(0);

    numGoals = size(goalPoses,1);
    q_guess = q_home;
    IKPenaltyValue = 50;

    for i = 1:numGoals
        % For each goal pose, set up the pose constraint and solve IK.
        pos_goal   = goalPoses(i,1:3);
        axang_goal = goalPoses(i,4:7);
        T_goal     = trvec2tform(pos_goal) * axang2tform(axang_goal);
        poseTgt    = constraintPoseTarget(robot.BodyNames{end});
        poseTgt.TargetTransform = T_goal;
        [q_sol, solInfo] = gik(q_guess, poseTgt, jointBounds);

        if solInfo.Status ~= "success"
            total_reachabilityPenalty = total_reachabilityPenalty + IKPenaltyValue;
            disp('ik failed!!')
        end
        try
            rrtPath = planner.plan(q_guess, q_sol);
        catch
            rrtPath = [];
        end
        if isempty(rrtPath)
            total_reachabilityPenalty = total_reachabilityPenalty + collisionPenaltyValue;
            disp('No Path found!')
        else
            for j = 1:size(rrtPath,1)
                q_path = rrtPath(j,:);
                if checkCollision(robot, q_path, collisionCylinders, 'SkippedSelfCollisions', skiplist)
                    total_reachabilityPenalty = total_reachabilityPenalty + collisionPenaltyValue;
                    disp('Collision in path!')
                    break;
                end
            end
        end
        % Compute error between achieved and target end-effector pose.
        T_ee = getTransform(robot, q_sol, robot.BodyNames{end});
        T_err = inv(T_goal) * T_ee;
        p_err = T_err(1:3,4);
        R_err = T_err(1:3,1:3);
        theta_err = acos(min(max((trace(R_err)-1)/2, -1), 1));
        pose_error = sqrt(norm(p_err)^2 + theta_err^2);
        totalPoseError = totalPoseError + pose_error;
        q_guess = q_sol;
    end
    reachability = totalPoseError + total_reachabilityPenalty;
    foldability = sum_volume + foldingMotion + bestminRollCollisionPenalty;

    %%FINAL COMBINATION

    % Weights for the different components.
    designWeight = 1;
    reachabilityWeight  = 1;
    foldabilityWeight = 1;

    % Final objective combines design cost and penalties.
    objective = designWeight * sum_design + reachabilityWeight * reachability + foldabilityWeight * foldability;
    %objective = reachabilityWeight * sum_design + foldabilityWeight * sum_volume + trajectoryWeight * trajectoryObjective;
    %objective = penaltyWeight * volumeCandidate + reachabilityWeight * sum_design + foldabilityWeight * sum_volume + trajectoryWeight * trajectoryObjective;

    candidateStruct.q_candidate = bestCandidateLocal;
    candidateStruct.value = objective;  % Use full objective, not isolated metric!
    
    % Send candidate info to the DataQueue to update the global best candidate.
     send(dq, struct(...
      'q_candidate',    bestCandidateLocal, ...
      'value',          objective, ...
      'designSum',      designWeight      * sum_design, ...
      'reachability',   reachabilityWeight* reachability, ...
      'foldability',    foldabilityWeight * foldability, ...
      'j8', bestJ8, ...
      'j10', bestJ10 ...
    ));
    
    fprintf('Objective: %.4f, Design Sum: %.4f, Reachability: %.4f, Foldability: %.4f\n', ...
            objective, designWeight * sum_design, reachabilityWeight * reachability, foldabilityWeight * foldability);

    % fprintf('Objective: %.4f, Volume Candidate: %.4f, Design Sum: %.4f\n, Volume Sum: %.4f, Trajectory penalty: %.4f', ...
    %         objective, penaltyWeight * volumeCandidate, sum_design, sum_volume, trajectoryObjective);
end