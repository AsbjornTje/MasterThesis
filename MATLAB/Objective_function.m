function cost = Objective_function(x)
    % x is a structure with fields: l1, a1, alpha1, l2, a2, alpha2, ... etc.
    
    % Example: extract link lengths (assuming two joints for demonstration)
    l1 = x.l1;
    l2 = x.l2;
    
    % Compute base objective: total link length weighted by beta1 
    beta1 = 1.0;
    baseCost = beta1 * (l1 + l2);
    
    % Compute forward kinematics to get joint positions and end-effector pose
    [jointPos, p_ee] = computeForwardKinematics(x);
    
    % Initialize penalty
    penalty = 0;
    
    % (i) Foldability / Bounding Box Constraint
    L_target = 1.0;  % Example value
    W_target = 0.5;
    H_target = 0.5;
    x_range = max(jointPos(:,1)) - min(jointPos(:,1));
    y_range = max(jointPos(:,2)) - min(jointPos(:,2));
    z_range = max(jointPos(:,3)) - min(jointPos(:,3));
    if x_range > L_target
        penalty = penalty + 100 * (x_range - L_target)^2;
    end
    if y_range > W_target
        penalty = penalty + 100 * (y_range - W_target)^2;
    end
    if z_range > H_target
        penalty = penalty + 100 * (z_range - H_target)^2;
    end
    
    % (ii) End-Effector Pose Error (split into position and orientation)
    % p_goal defined globally or passed via persistent variable, etc.
    global p_goal eps_pos eps_ori;
    posError = norm(p_ee.pos - p_goal.pos);
    oriError = norm(p_ee.ori - p_goal.ori);
    if posError > eps_pos
        penalty = penalty + 1000 * (posError - eps_pos)^2;
    end
    if oriError > eps_ori
        penalty = penalty + 1000 * (oriError - eps_ori)^2;
    end
    
    % (iii) Dexterity Constraint: Ensure manipulability index is above threshold
    J = computeJacobian(x);
    mu = abs(det(J));
    mu_min = 0.1;  % example threshold
    if mu < mu_min
        penalty = penalty + 1000 * (mu_min - mu)^2;
    end
    
    % (iv) Collision Avoidance using collision boxes (assume function collisionCheck returns a minimum distance)
    d_safe = 0.05; % minimum safe distance
    d_min = collisionCheck(jointPos); % user-defined function
    if d_min < d_safe
        penalty = penalty + 1000 * (d_safe - d_min)^2;
    end
    
    % (v) Load Carrying Capacity
    tau_required = computeRequiredTorque(x);
    if tau_required < (m_min * 9.81)  % m_min defined externally
        penalty = penalty + 1000 * ((m_min * 9.81) - tau_required)^2;
    end
    
    % Final cost is the sum of the base cost and the penalties.
    cost = baseCost + penalty;
end
