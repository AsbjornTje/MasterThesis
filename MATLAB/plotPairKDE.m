function stop = plotPairKDE(results,state,varX,varY)
% plotPairKDE   2-D Gaussian-KDE + scatter for any two bayesopt vars
%
% Usage:
%   'PlotFcn', {@(r,s)plotPairKDE(r,s,'d7','d8')}
%
% – initial: flat zero → uniform color  
% – iteration: sum Gaussian bumps from each sample → smooth, circular  
%              hot-spots around clusters of (varX,varY)  
% – done:     hold final view

    stop = false;

    %—— 1) Build grid & bandwidth once per (varX,varY) combo ——
    persistent xi yi Xg Yg sigmaX sigmaY lastX lastY
    if isempty(lastX) || ~strcmp(lastX,varX) || ~strcmp(lastY,varY)
        % store the pair so we re-build only if they change
        lastX = varX;
        lastY = varY;

        % look up their bounds
        vds   = results.VariableDescriptions;
        names = {vds.Name};
        idxX  = find(strcmp(names,varX),1);
        idxY  = find(strcmp(names,varY),1);
        rX    = vds(idxX).Range;   % [min max]
        rY    = vds(idxY).Range;

        % grid resolution (you can tweak)
        nG = 100;
        xi = linspace(rX(1), rX(2), nG);
        yi = linspace(rY(1), rY(2), nG);
        [Xg, Yg] = meshgrid(xi, yi);

        % set bandwidth ≈1/10 of each span
        sigmaX = (rX(2)-rX(1))/10;
        sigmaY = (rY(2)-rY(1))/10;
    end

    switch state
        case 'initial'
            figure('Name',sprintf('%s vs %s KDE',varX,varY),...
                   'NumberTitle','off');
            % flat prior
            imagesc(xi, yi, zeros(size(Xg)));
            set(gca,'YDir','normal');
            colormap(jet); caxis([0 1]); colorbar;
            xlabel(varX); ylabel(varY);
            title(sprintf('KDE (iteration 0): %s vs %s',varX,varY));
            hold on;

        case 'iteration'
            T = results.XTrace;
            if isempty(T), return, end

            x = T.(varX);  y = T.(varY);
            nS = numel(x);

            % build KDE by summing Gaussian bumps
            F = zeros(size(Xg));
            inv2sX = 1/(2*sigmaX^2);
            inv2sY = 1/(2*sigmaY^2);
            for i = 1:nS
                dx = Xg - x(i);
                dy = Yg - y(i);
                F  = F + exp(-dx.^2*inv2sX - dy.^2*inv2sY);
            end

            % normalize so that max(F)=1
            F = F / max(F(:));

            % plot density + overlay points
            clf;
            imagesc(xi, yi, F);
            set(gca,'YDir','normal');
            colormap(jet); caxis([0 1]); colorbar;
            hold on;
            scatter(x, y, 20, 'k','filled');

            xlabel(varX); ylabel(varY);
            title(sprintf('KDE (iteration %d): %s vs %s', nS, varX, varY));
            drawnow;

        case 'done'
            hold off;
    end
end
