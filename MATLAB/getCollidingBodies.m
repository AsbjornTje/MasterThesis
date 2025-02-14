function collidingPairs = getCollidingBodies(robot, q)
    % getCollidingBodies checks the robot for self-collisions in configuration q
    % and returns a cell array of colliding body name pairs.
    
    numBodies = numel(robot.Bodies);
    collidingPairs = {};  % initialize cell array to hold colliding pairs
    
    % Iterate over all unique body pairs.
    for i = 1:numBodies-1
        body1 = robot.Bodies{i};
        % If a body has no collision geometry, skip it.
        if isempty(body1.Collisions)
            continue;
        end
        
        for j = i+1:numBodies
            body2 = robot.Bodies{j};
            if isempty(body2.Collisions)
                continue;
            end
            
            % Initialize flag for current body pair.
            isPairColliding = false;
            
            % Check each collision object in body1 against each in body2.
            for k = 1:numel(body1.Collisions)
                % Transform collision geometry for body1 into world frame
                T1 = getTransform(robot, q, robot.Bodies{i}.Parent.Name, body1.Name);
                colObj1 = body1.Collisions{k};
                colObj1.Pose = T1 * colObj1.Pose;
                
                for m = 1:numel(body2.Collisions)
                    T2 = getTransform(robot, q, robot.Bodies{j}.Parent.Name, body2.Name);
                    colObj2 = body2.Collisions{m};
                    colObj2.Pose = T2 * colObj2.Pose;
                    
                    % Check collision between these two objects.
                    % checkCollision for two collision objects returns a boolean.
                    if checkCollision(colObj1, colObj2)
                        isPairColliding = true;
                        break;
                    end
                end
                if isPairColliding
                    break;
                end
            end
            
            if isPairColliding
                collidingPairs(end+1,:) = {body1.Name, body2.Name}; %#ok<AGROW>
            end
        end
    end
end
