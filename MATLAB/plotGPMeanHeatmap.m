function stop = plotGPMeanHeatmap(results, state, varX, varY)
% plotGPMeanHeatmap  GP predicted‐mean contour + sample scatter
%   stop = plotGPMeanHeatmap(results, state, varX, varY)
%
% Hook into bayesopt via:
%   'PlotFcn', {@(r,s)plotGPMeanHeatmap(r,s,'d10','d12')}

    stop = false;

    %—— Build grid once per (varX,varY) ——%
    persistent xi yi Xg Yg lastX lastY
    if isempty(lastX) || ~strcmp(lastX,varX) || ~strcmp(lastY,varY)
        lastX = varX;  lastY = varY;
        vds   = results.VariableDescriptions;
        names = {vds.Name};
        iX = find(strcmp(names,varX),1);
        iY = find(strcmp(names,varY),1);
        rX = vds(iX).Range;
        rY = vds(iY).Range;

        nG = 100;
        xi = linspace(rX(1), rX(2), nG);
        yi = linspace(rY(1), rY(2), nG);
        [Xg, Yg] = meshgrid(xi, yi);
    end

    switch state
      case 'initial'
        figure('Name',sprintf('GP Mean: %s vs %s',varX,varY),...
               'NumberTitle','off');
        xlabel(varX); ylabel(varY);
        title('GP Predicted Mean');
        return;

      case 'iteration'
        % bring up & clear
        h = findall(groot,'Type','figure','Name',sprintf('GP Mean: %s vs %s',varX,varY));
        if isempty(h)
            figure('Name',sprintf('GP Mean: %s vs %s',varX,varY),'NumberTitle','off');
        else
            figure(h(1)); clf;
        end

      otherwise  % 'done'
        return;
    end

    %—— Build a full-D table for prediction ——%
    bestPt = results.XAtMinObjective;  % 1×D table of best so far
    nPts   = numel(Xg);
    Tbl    = repmat(bestPt, nPts, 1);
    Tbl.(varX) = reshape(Xg, nPts, 1);
    Tbl.(varY) = reshape(Yg, nPts, 1);

    %—— Predict the GP mean at each grid point ——%
    mu = predictObjective(results, Tbl);
    Mu = reshape(mu, size(Xg));

    %—— Plot contour + scatter ——%
    contourf(xi, yi, Mu, 20, 'LineColor','none');
    colormap(jet);
    colorbar('EastOutside','LabelString','Predicted Mean');
    set(gca,'YDir','normal');
    hold on;

    % overlay actual samples
    T = results.XTrace;
    scatter(T.(varX), T.(varY), 30, 'k', 'filled');

    xlabel(varX); ylabel(varY);
    title(sprintf('Iter %d: GP Mean of Objective', height(T)));
    drawnow;
end
