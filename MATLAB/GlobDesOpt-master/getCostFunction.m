%Cost function to minimize
function [Cost] = getCostFunction(x,Robots,Indexes,acceptRate,Npnts_WS,cost_fcn,plot_en)


Robots = generateRobots(x,Robots,Indexes);
dual_arm_copy = Indexes{1}.dual_arm_copy;

    if cost_fcn == "Minimize_d_with_goals"
        % --- Part 1: Objective on the d parameter ---
        % Extract indices for the d parameters (assumed to be in Indexes{1}.d)
        d_indices = Indexes{1}.d;
        d_values = x(d_indices);
        cost_d = sum(d_values);  % or use another measure (e.g., max, weighted sum)
        
        % --- Part 2: Reachability penalty ---
        % Compute the workspace (using your existing function)
        [~, shps, ~, ~, ~] = getWSVolumes(Robots, Indexes{1}.dual_arm_copy, acceptRate, Npnts_WS, cost_fcn, plot_en);
        % Here, we assume the first shape represents the reachable workspace
        workspaceShape = shps{1};

        goalPoses=[4, 1, 2;
            6, 4, 3;
            ];
        
        penalty = 0;

    % Check if the workspace shape is empty
    if isempty(workspaceShape.Points)
        % Assign a high penalty cost if the shape is empty
        penalty = 1e6;
    else
        penalty = 0;
        for i = 1:size(goalPoses,1)
            if ~inShape(workspaceShape, goalPoses(i,1), goalPoses(i,2), goalPoses(i,3))
                penalty = penalty + 100;  % Adjust penalty factor as needed
            end
        end
    end
        
        lambda = 1;  % Weight factor for penalty
        cost_d = sum(x(Indexes{1}.d));  % Example: sum of d parameters
        Cost = cost_d + lambda * penalty;
        
    else
        % Your original workspace-based cost function (if needed)
        [~,~,Vs,Safety,ave_dext] = getWSVolumes(Robots, Indexes{1}.dual_arm_copy, acceptRate, Npnts_WS, cost_fcn, plot_en);
        if length(Vs) > 1
            V = Vs(3);
        else
            V = Vs(1);
        end
        V = ave_dext * V;
        SafetyVolume = max(V * Safety, 1e-16);
        Cost = -log10(SafetyVolume);
    end

% disp("| Volume | "+num2str(Vs(3))+...
%     " | Saefty | "+num2str(Safety));
end

