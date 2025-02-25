function collisionDetected = checkAllEnvironmentCollisions(robot, q, collisionObjs)
    collisionDetected = false;
    for idx = 1:length(collisionObjs)
        if checkCollision(robot, q, collisionObjs{idx}, 'SkippedSelfCollision', 'parent')
            collisionDetected = true;
            return;
        end
    end
end
