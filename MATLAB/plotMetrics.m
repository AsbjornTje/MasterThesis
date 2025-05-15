function stop = plotMetrics(~, state)
    % plotReachVsFold  Live plot of reachability & foldability vs iteration
    stop = false;
    persistent fig
    global reachTrace foldTrace

    if strcmp(state,'initial')
        fig = figure('Name','Reach vs Fold','NumberTitle','off');
        return
    end
    if ~strcmp(state,'iteration')
        return
    end

    % bring up or recreate
    if isempty(fig) || ~ishandle(fig)
        fig = figure('Name','Reach vs Fold','NumberTitle','off');
    else
        figure(fig); clf;
    end

    it = (1:numel(reachTrace))';

    plot(it, reachTrace, '-s', 'LineWidth',1.5, 'DisplayName','Reachability'); hold on;
    plot(it, foldTrace,  '-^', 'LineWidth',1.5, 'DisplayName','Foldability');

    xlabel('BO Iteration');
    ylabel('Penalty');
    title('Reachability & Foldability vs. Iteration');
    legend('Location','best');
    grid on;
    drawnow;
end