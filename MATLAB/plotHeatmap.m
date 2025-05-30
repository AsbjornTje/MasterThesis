function stop = plotHeatmap(results,state,varX,varY)
    stop = false;

    % Build grid & bandwidth once per (varX,varY) 
    persistent xi yi Xg Yg sigmaX sigmaY lastX lastY
    if isempty(lastX) || ~strcmp(lastX,varX) || ~strcmp(lastY,varY)
        lastX = varX;  lastY = varY;

        vds   = results.VariableDescriptions;
        names = {vds.Name};
        idxX  = find(strcmp(names,varX),1);
        idxY  = find(strcmp(names,varY),1);
        rX    = vds(idxX).Range;
        rY    = vds(idxY).Range;

        nG = 100;
        xi = linspace(rX(1), rX(2), nG);
        yi = linspace(rY(1), rY(2), nG);
        [Xg, Yg] = meshgrid(xi, yi);

        sigmaX = (rX(2)-rX(1))/10;
        sigmaY = (rY(2)-rY(1))/10;
    end

    % Figure window management
    persistent figHeat
    figName = sprintf('%s vs %s Heatmap',varX,varY);
    switch state
      case 'initial'
        figHeat = figure('Name',figName,'NumberTitle','off', ...
                         'Units','pixels','Position',[100 100 1800 800]);
        return
      case 'iteration'
        if isempty(figHeat) || ~ishandle(figHeat)
          figHeat = figure('Name',figName,'NumberTitle','off');
        else
          figure(figHeat);
        end
        clf;
      otherwise
        return
    end

    %Fetch data
    T = results.XTrace;            
    Y = results.ObjectiveTrace;

    % Build weighted heatmap
    inv2sX = 1/(2*sigmaX^2);
    inv2sY = 1/(2*sigmaY^2);
    F = zeros(size(Xg));
    Ymin = min(Y);  Ymax = max(Y);
    w    = (Ymax - Y) ./ (Ymax - Ymin + eps);
    for i = 1:numel(w)
        dx = Xg - T.(varX)(i);
        dy = Yg - T.(varY)(i);
        F  = F + w(i)*exp(-dx.^2*inv2sX - dy.^2*inv2sY);
    end
    F = F / max(F(:));

    imagesc(xi, yi, F);
    set(gca,'YDir','normal');
    colormap(jet);
    caxis([0 1]);
    colorbar;
    hold on;

    %Highlight points by their rank in Y
    n = height(T);
    [~, order] = sort(Y,'ascend'); 
    top5   = order(1 : min(5,n));
    next10 = order(min(5,n)+1 : min(15,n));
    rest   = order(min(15,n)+1 : n);

    hRest   = scatter(T.(varX)(rest),   T.(varY)(rest),   30, 'k',   'filled', 'MarkerEdgeColor','w');
    hNext10 = scatter(T.(varX)(next10), T.(varY)(next10), 30, [0.5 0.5 0.5], 'filled', 'MarkerEdgeColor','w');
    hTop5   = scatter(T.(varX)(top5),   T.(varY)(top5),   30, 'w',   'filled', 'MarkerEdgeColor','w');

    % Add a legend for the point‐groups
    legend([hTop5,hNext10,hRest], ...
           {'Top 5','Next 10','Others'}, ...
           'Location','northeastoutside');

    xlabel(varX);
    ylabel(varY);
    title(sprintf('iter %d: %s vs %s', n, varX, varY));
    drawnow;
end
