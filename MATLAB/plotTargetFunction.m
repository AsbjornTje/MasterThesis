function stop = plotTargetFunction(results,state,varX,varY)
% plotTargetFunction  Contour of the TRUE objective over two vars + samples
%   stop = plotTargetFunction(results,state,varX,varY)
%   • results : BayesianOptimization object (has .ObjectiveFcn)
%   • state   : 'initial','iteration','done'
%   • varX    : name of the first variable (e.g. 'd10')
%   • varY    : name of the second variable (e.g. 'd12')

    stop = false;

    % Debug: see when we get called
    fprintf('[plotTargetFunction] state = %s\n',state);

    %% 1) Build grid & true‐objective values once per (varX,varY)
    persistent xi yi Xg Yg Z lastX lastY baseline vds
    if isempty(lastX) || ~strcmp(lastX,varX) || ~strcmp(lastY,varY)
        lastX = varX;  lastY = varY;

        % grab the ranges from the optimizer itself
        vds   = results.VariableDescriptions;
        names = {vds.Name};
        idxX  = find(strcmp(names,varX),1);
        idxY  = find(strcmp(names,varY),1);
        rX    = vds(idxX).Range;
        rY    = vds(idxY).Range;

        % make a regular grid
        nG = 100;
        xi = linspace(rX(1), rX(2), nG);
        yi = linspace(rY(1), rY(2), nG);
        [Xg, Yg] = meshgrid(xi, yi);

        % compute a “baseline” table of midpoints for all other vars
        baseline = table();
        for k = 1:numel(vds)
            nm = vds(k).Name;
            if strcmp(nm,varX) || strcmp(nm,varY), continue; end
            baseline.(nm) = mean(vds(k).Range);
        end

        % evaluate the *true* objective at each grid point
        Z = zeros(size(Xg));
        objHandle = results.ObjectiveFcn;  % the original handle you passed in
        for i = 1:numel(Xg)
            T = baseline;
            T.(varX) = Xg(i);
            T.(varY) = Yg(i);
            % call the real objective
            Z(i) = objHandle(T);
        end
        Z = reshape(Z, size(Xg));
    end

    %% 2) On initial or iteration, draw/update the figure
    if strcmp(state,'initial') || strcmp(state,'iteration')
        figName = sprintf('Target: %s vs %s', varX, varY);
        hFig = findobj('Type','figure','Name',figName);
        if isempty(hFig)
            hFig = figure('Name',figName,'NumberTitle','off');
        else
            figure(hFig); clf;
        end
        hold on;

        % draw the contour of the true surface
        contourf(xi, yi, Z, 30, 'LineColor','none');
        colormap(jet); colorbar;
        set(gca,'YDir','normal');
        xlabel(varX); ylabel(varY);
        title(figName);

        % overlay the actual sampled points
        T = results.XTrace;
        if ~isempty(T)
            scatter(T.(varX), T.(varY), 20, 'k','filled');
        end
        drawnow;
    end
    % on 'done' we simply leave the final figure up
end
