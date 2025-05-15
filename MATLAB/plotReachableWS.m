function [vol, K, Pfree] = plotReachableWS(robot, numSamples, collisionCylinders, q_home)
% plotReachableWS   Workspace hull + obstacles + robot
%
%   [vol, K, P] = plotReachableWS(robot, numSamples, collisionCylinders, q_home)
%
% Inputs:
%   robot              – your optimized RigidBodyTree
%   numSamples         – number of random joint‐samples (e.g. 5000–10000)
%   collisionCylinders – cell array of collisionCylinder objects
%   q_home             – a 1×nJ joint vector (e.g. homeConfiguration(robot))
%
% Outputs:
%   vol – convex‐hull volume (m³)
%   K   – hull triangulation indices
%   P   – numSamples×3 end‐effector positions

   % Build skip list for self‐collision test
    adj = [robot.Base.Name, robot.BodyNames];
    skip = cell(robot.NumBodies,2);
    for i = 1:robot.NumBodies
        skip(i,:) = {adj{i}, adj{i+1}};
    end

    % Joint‐limit bounds
    qc = homeConfiguration(robot);
    nJ = numel(qc);
    bounds = zeros(nJ,2);
    for j = 1:nJ
        bounds(j,:) = robot.Bodies{j}.Joint.PositionLimits;
    end

    % Sample & filter
    Pfree = zeros(0,3);
    eeName = robot.BodyNames{end};
    for i = 1:numSamples
        q = bounds(:,1)' + rand(1,nJ).*diff(bounds,1,2)';
        if checkCollision(robot,q,'SkippedSelfCollisions',skip), continue, end
        envHit = false;
        for c=1:numel(envCylinders)
            if ~isempty(checkCollision(robot,q,envCylinders{c},...
                                       'SkippedSelfCollisions',skip))
                envHit = true; break
            end
        end
        if envHit, continue, end
        T = getTransform(robot,q,eeName);
        Pfree(end+1,:) = tform2trvec(T);
    end

    % Deduplicate
    Pfree = unique(Pfree,'rows');

    % Prepare figure
    figure('Name','Feasible Reachable Workspace','NumberTitle','off');
    ax = axes; hold(ax,'on'); view(ax,3); grid(ax,'on'); axis(ax,'equal');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

    % Try convex hull if enough points
    vol = NaN; K = [];
    if size(Pfree,1)>=4 && rank(bsxfun(@minus,Pfree,mean(Pfree,1)))==3
        try
            [K,vol] = convhull(Pfree(:,1),Pfree(:,2),Pfree(:,3));
            trisurf(K, Pfree(:,1),Pfree(:,2),Pfree(:,3), ...
                    'FaceColor',[0.2,0.6,1], 'FaceAlpha',0.2, 'EdgeColor','none');
        catch
            warning('Convex hull failed—falling back to scatter.');
            scatter3(ax, Pfree(:,1),Pfree(:,2),Pfree(:,3), 8, '.', 'MarkerEdgeAlpha',0.5);
        end
    else
        warning('Not enough unique 3D points for a hull—showing scatter only.');
        scatter3(ax, Pfree(:,1),Pfree(:,2),Pfree(:,3), 8, '.', 'MarkerEdgeAlpha',0.5);
    end

    % Plot cylinders
    for c = 1:numel(collisionCylinders)
        show(collisionCylinders{c}, 'Parent', ax);
    end

    % Plot robot
    show(robot, q_home, 'Parent', ax, 'PreservePlot', false);

    if isnan(vol)
        title(ax,'Reachable Workspace (scatter only)');
    else
        title(ax,sprintf('Reachable Workspace (vol=%.2f m³)',vol));
    end
    hold(ax,'off');
end