function error = computeGoalPoseError(Robots, goalPoses)
    error = 0;
    for i = 1:size(goalPoses, 1)
        pos = goalPoses(i, 1:3);
        rot = goalPoses(i, 4:7);
        goalPose = trvec2tform(pos) * axang2tform(rot);  % Convert to homogeneous transform
        
        % Get current end-effector pose
        q = zeros(Robots{1}.m_n_jnts,1);  % Neutral joint config
        [currentPose, ~] = Robots{1}.getFwdKine(q, "ee");
        
        % Position error
        pos_error = norm(goalPose(1:3,4) - currentPose(1:3,4));
        
        % Orientation error using rotation matrix difference
        rot_error = norm(logm(goalPose(1:3,1:3)' * currentPose(1:3,1:3)), 'fro');
        
        error = error + pos_error + rot_error;
    end
    error = error / size(goalPoses, 1);  % Average error
end
