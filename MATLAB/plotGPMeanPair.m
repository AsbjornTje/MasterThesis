function stop = plotGPMeanPair(results,state,varX,varY)
% plotGPMeanPair   Contour of GP predicted mean over two vars + samples
%
%   stop = plotGPMeanPair(results,state,varX,varY)
%   • results : BayesianOptimization object
%   • state   : 'initial','iteration','done'
%   • varX/varY : names of the two variables to plot
%
% Hook it up via:
%   'PlotFcn', {@(r,s) plotGPMeanPair(r,s,'d10','d12')}

    stop = false;

    %% 1) Build grid & index mapping once
    persistent xi yi Xg Yg idxX idxY vds names
    if isempty(xi)
        vds   = results.VariableDescriptions;
        names = {vds.Name};

        idxX = find(strcmp(names,varX),1);
        idxY = find(strcmp(names,varY),1);

        rX = vds(idxX).Range;  % [min max]
        rY = vds(idxY).Range;

        nG = 100;
        xi = linspace(rX(1), rX(2), nG);
        yi = linspace(rY(1), rY(2), nG);
        [Xg, Yg] = meshgrid(xi, yi);
    end

    %% 2) On initial or iteration, compute & plot
    if strcmp(state,'initial') || strcmp(state,'iteration')
        figName = sprintf('GP Mean: %s vs %s', varX, varY);
        hFig = findobj('Type','figure','Name',figName);
        if isempty(hFig)
            hFig = figure('Name',figName,'NumberTitle','off');
        else
            figure(hFig); clf;
        end
        hold on;

        % Build a table of grid points, freezing other variables at best-so-far
        bestPt = results.XAtMinObjective;
        if isempty(bestPt)
            bestPt = results.XAtMinEstimatedObjective;
        end
        nPts = numel(Xg);
        Tbl = repmat(bestPt, nPts, 1);
        Tbl.(varX) = Xg(:);
        Tbl.(varY) = Yg(:);

        % Predict the GP mean at each grid location
        mu = predictObjective(results, Tbl);
        Mu = reshape(mu, size(Xg));

        % Draw contour of the actual predicted-mean values
        contourf(xi, yi, Mu, 30, 'LineColor','none');
        colormap(jet);
        cb = colorbar;
        ylabel(cb,'Predicted Mean');
        set(gca,'YDir','normal');
        xlabel(varX); ylabel(varY);
        title(sprintf('Iteration %d: GP Predicted Mean', height(results.XTrace)));

        % Overlay the sampled points
        T = results.XTrace;
        scatter(T.(varX), T.(varY), 20, 'k', 'filled');

        drawnow;
    end
    % nothing to do on 'done'
end
