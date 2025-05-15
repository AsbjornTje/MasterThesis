function plotReachVsGoal(robot, collisionCylinders, goalPoses)
    % 1) Sample large workspace
    numSamp = 8000;
    Q = sampleRandomConfigs(robot, numSamp);

    eeName = robot.BodyNames{end};
    P = zeros(numSamp,3);
    for i=1:numSamp
        T = getTransform(robot, Q(i,:), eeName);
        P(i,:) = tform2trvec(T);
    end

    % 2) compute distance to nearest goal
    G = goalPoses(:,1:3);
    dists = min( pdist2(P, G), [], 2 );  % numSamp×1

    % 3) plot
    figure('Name','Workspace colored by distance to nearest goal','NumberTitle','off');
    scatter3(P(:,1),P(:,2),P(:,3),6,dists,'filled');
    colormap(jet); colorbar;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Reachable Workspace: color = dist to closest goal');
    hold on;
    % overlay goal positions
    plot3(G(:,1),G(:,2),G(:,3),'kp','MarkerSize',12,'MarkerFaceColor','w');
    legend('Workspace samples','Goal poses','Location','best');
    axis equal; grid on; view(3);
    
    % 4) overlay cylinders
    for k=1:numel(collisionCylinders)
        h = show(collisionCylinders{k},'Parent',gca);
        if isprop(h,'FaceAlpha')
            h.FaceColor = [1 0 0];
            h.FaceAlpha = 0.2;
            h.EdgeColor = 'none';
        end
    end
    hold off;
end

function Q = sampleRandomConfigs(robot, N)
    qc = homeConfiguration(robot);
    nJ = numel(qc);
    bounds = zeros(nJ,2);
    for i=1:nJ
        bounds(i,:) = robot.Bodies{i}.Joint.PositionLimits;
    end
    Q = rand(N,nJ);
    for j=1:nJ
        Q(:,j) = bounds(j,1) + (bounds(j,2)-bounds(j,1)) * Q(:,j);
    end
end
