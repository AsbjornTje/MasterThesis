function plotParamIsolation(results, dhparams_full, jointTypes, goalRegion, bestQ)
    % Define variables and axes
    vars   = {'a6','d7','d8','d9','d10','d11','d12'};
    axesNm = {'X','Y','Z'};
    nVars  = numel(vars);
    nPts   = 50;

    % Pull ranges
    vds   = results.VariableDescriptions;
    names = {vds.Name};

    % Preallocate
    gridVals = cell(nVars,1);
    volX = nan(nVars,nPts);
    volY = nan(nVars,nPts);
    volZ = nan(nVars,nPts);

    % Sweep each DH parameter
    for k = 1:nVars
        varName = vars{k};
        idx     = find(strcmp(names,varName),1);
        range   = vds(idx).Range;
        vals    = linspace(range(1), range(2), nPts);
        gridVals{k} = vals;

        for i = 1:nPts
            % Build DH with only varName changed
            dh = dhparams_full;
            switch varName
                case 'a6',  dh(6,1)  = vals(i);
                case 'd7',  dh(7,3)  = vals(i);
                case 'd8',  dh(8,3)  = vals(i);
                case 'd9',  dh(9,3)  = vals(i);
                case 'd10', dh(10,3) = vals(i);
                case 'd11', dh(11,3) = vals(i);
                case 'd12', dh(12,3) = vals(i);
            end
            % Rebuild at bestQ
            [robot, collisionData] = createRobotCollisionModel(dh, jointTypes, bestQ);
            % Measure protrusion at fixed bestQ
            [vX, vY, vZ] = Protruding_Volume(robot, collisionData, bestQ, goalRegion, 1000, false);
            volX(k,i) = vX;
            volY(k,i) = vY;
            volZ(k,i) = vZ;
        end
    end

    % Create combined figure
    f = figure('Name','DH Param Isolation','NumberTitle','off', ...
               'Units','pixels','Position',[100 100 1800 800]);
    sgtitle('DH Parameter Sensitivity by Axis','FontSize',16);

    % Loop through each var and axis
    for k = 1:nVars
        for a = 1:3
            idx = (k-1)*3 + a;
            ax = subplot(nVars,3,idx);
            switch a
                case 1
                    plot(gridVals{k}, volX(k,:), 'LineWidth',1.5);
                case 2
                    plot(gridVals{k}, volY(k,:), 'LineWidth',1.5);
                case 3
                    plot(gridVals{k}, volZ(k,:), 'LineWidth',1.5);
            end
            xlabel(vars{k}, 'FontSize',10);
            ylabel(['vol_' axesNm{a}], 'FontSize',10);
            title([vars{k} ' 	o ' axesNm{a}], 'FontSize',11);
            grid(ax, 'on');
            ax.FontSize = 9;
        end
    end
end
