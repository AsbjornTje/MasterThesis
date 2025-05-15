function plotDHtoVolume(results, dhparams_full, jointTypes, goalRegion, q_home)
% plotDHtoVolume   Sensitivity‐sweep of each DH var → protruding volume per axis
%
%   plotDHtoVolume(results, dhparams_full, jointTypes, goalRegion, q_home)
%
% Produces a 3×7 figure: rows = volX, volY, volZ; columns = a6, d7…d12

    % 1) Best‐found DH settings from BO
    best = results.XAtMinObjective;  % table with columns a6,d7…d12

    vars  = {'a6','d7','d8','d9','d10','d11','d12'};
    nVars = numel(vars);
    nPts  = 50;   % sweep resolution

    % 2) Prepare storage
    volX = nan(nVars, nPts);
    volY = nan(nVars, nPts);
    volZ = nan(nVars, nPts);
    gridVals = cell(nVars,1);

    vds   = results.VariableDescriptions;
    names = {vds.Name};

    % 3) Sweep each variable
    for k = 1:nVars
        varName = vars{k};
        idx     = find(strcmp(names,varName),1);
        range   = vds(idx).Range;     % [min max]
        vals    = linspace(range(1), range(2), nPts);
        gridVals{k} = vals;

        for i = 1:nPts
            % 3a) build DH for this sweep point
            dh = dhparams_full;
            for j = 1:nVars
                nm = vars{j};
                if j==k
                    v = vals(i);
                else
                    v = best.(nm);
                end
                switch nm
                  case 'a6',  dh(6,1)  = v;
                  case 'd7',  dh(7,3)  = v;
                  case 'd8',  dh(8,3)  = v;
                  case 'd9',  dh(9,3)  = v;
                  case 'd10', dh(10,3) = v;
                  case 'd11', dh(11,3) = v;
                  case 'd12', dh(12,3) = v;
                end
            end

            % 3b) compute independent minima for X, Y, Z volumes
            [vx_min, vy_min, vz_min] = computeProtrusionComponents( ...
                                      dh, jointTypes, goalRegion, q_home);
            volX(k,i) = vx_min;
            volY(k,i) = vy_min;
            volZ(k,i) = vz_min;
        end
    end

    % 4) Plot 3×7 grid
    figure('Name','DH Sensitivity per Axis','NumberTitle','off',...
        'Units','pixels','Position',[100 100 1800 800]);
    for k = 1:nVars
        % X‐volume
        subplot(3,nVars,k);
        plot(gridVals{k}, volX(k,:), 'LineWidth',1.5);
        xlabel(vars{k}); ylabel('vol\_X');
        title([vars{k} ' → X']);
        grid on;

        % Y‐volume
        subplot(3,nVars,nVars + k);
        plot(gridVals{k}, volY(k,:), 'LineWidth',1.5);
        xlabel(vars{k}); ylabel('vol\_Y');
        title([vars{k} ' → Y']);
        grid on;

        % Z‐volume
        subplot(3,nVars,2*nVars + k);
        plot(gridVals{k}, volZ(k,:), 'LineWidth',1.5);
        xlabel(vars{k}); ylabel('vol\_Z');
        title([vars{k} ' → Z']);
        grid on;
    end
end


%% Helper: scan roll angles and return min volX, volY, volZ
function [vx_min, vy_min, vz_min] = computeProtrusionComponents( ...
                                    dh, jointTypes, goalRegion, q_home)
    % Rebuild robot & collisionData
    [robot, collisionData] = createRobotCollisionModel(dh, jointTypes, q_home);

    % Build skiplist (same as in your ObjectiveFcn_Full)
    adj = [robot.Base.Name, robot.BodyNames];
    skiplist = cell(robot.NumBodies,2);
    for m = 1:robot.NumBodies
        skiplist(m,:) = {adj{m}, adj{m+1}};
    end
    skiplist(end+1,:) = {'body6','body8'};

    % Fixed folded posture
    q_fold = [0 pi/2 pi/2 -0.7 0 pi 0 pi 0];
    rollIdx8  = 5;
    rollIdx10 = 7;
    step = deg2rad(30);   % coarse for speed

    % Initialize
    vx_min = inf;
    vy_min = inf;
    vz_min = inf;

    % Sweep both roll joints
    for r8 = -pi:step:pi
      for r10 = -pi:step:pi
        q = q_fold;
        q(rollIdx8)  = r8;
        q(rollIdx10) = r10;
        [vX, vY, vZ] = Protruding_Volume( ...
                          robot, collisionData, q, goalRegion, 1000, false);
        % track minima individually
        if vX < vx_min, vx_min = vX; end
        if vY < vy_min, vy_min = vY; end
        if vZ < vz_min, vz_min = vZ; end
      end
    end
end
