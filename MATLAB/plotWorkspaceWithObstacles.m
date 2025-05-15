function plotWorkspaceWithObstacles(robot, collisionCylinders)
% plotWorkspaceWithObstacles  Visualize reachable workspace and obstacles
%   plotWorkspaceWithObstacles(robot, collisionCylinders)
%   • robot               : RigidBodyTree
%   • collisionCylinders  : cell array of collision objects
%
% Samples the robot’s reachable workspace by random joint-configuration
% sampling, plots the end-effector scatter, and overlays collision cylinders.

    % 1) Sampling settings
    numSamples = 5000;
    rng(0);

    % 2) Extract joint limits
    qHome = homeConfiguration(robot);
    nJ = numel(qHome);
    jointBounds = zeros(nJ,2);
    for i = 1:nJ
        joint = robot.Bodies{i}.Joint;
        jointBounds(i,:) = joint.PositionLimits;
    end

    % 3) Sample random configurations
    Q = zeros(numSamples, nJ);
    for j = 1:nJ
        lb = jointBounds(j,1);
        ub = jointBounds(j,2);
        Q(:,j) = lb + (ub - lb) * rand(numSamples,1);
    end

    % 4) Compute end-effector positions
    eeName = robot.BodyNames{end};
    positions = zeros(numSamples,3);
    for k = 1:numSamples
        Tform = getTransform(robot, Q(k,:), eeName);
        positions(k,:) = tform2trvec(Tform);
    end

    % 5) Plot workspace scatter
    fig = figure('Name','Reachable Workspace','NumberTitle','off');
    scatter3(positions(:,1), positions(:,2), positions(:,3), 4, '.', 'MarkerEdgeAlpha',0.3);
    hold on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Reachable Workspace with Collision Cylinders');
    axis equal;
    grid on;

    % 6) Overlay collision cylinders with transparency
    ax = gca;
    for c = 1:numel(collisionCylinders)
        hCyl = show(collisionCylinders{c}, 'Parent', ax);
        % apply face transparency if supported
        if isprop(hCyl,'FaceAlpha')
            hCyl.FaceColor = [1 0 0];
            hCyl.FaceAlpha = 0.2;
            hCyl.EdgeColor = 'none';
        end
    end

    view(3);
    hold off;
end
