function stop = plotDesignSum(results,state)
% plotDesignSum  Custom BayesOpt plot: Design‐sum vs. iteration
%    stop = plotDesignSum(results,state)
%    • results : BayesianOptimization object
%    • state   : 'initial','iteration','done'
% Plots sum(abs(d8)+abs(d10)+abs(d12)) over the past samples.

    stop = false;
    persistent figDS
    figName = 'Design Sum vs. Iteration';

    switch state
      case 'initial'
        % create the figure window
        figDS = figure('Name',figName,'NumberTitle','off', ...
            'Units','pixels','Position',[100 100 1800 800]);
        return

      case 'iteration'
        % bring up or recreate & clear it
        if isempty(figDS) || ~ishandle(figDS)
            figDS = figure('Name',figName,'NumberTitle','off');
        else
            figure(figDS); clf;
        end

        % get the table of past samples
        T = results.XTrace;
        % ensure the design vars exist
        if all(ismember({'d8','d10','d12'}, T.Properties.VariableNames))
            % compute the design sum per iteration
            ds = abs(T.d8) + abs(T.d10) + abs(T.d12);
            plot(1:height(T), ds, '-o', 'LineWidth',1.5);
            xlabel('Iteration');
            ylabel('Design Sum = |d8|+|d10|+|d12|');
            title(figName);
            grid on;
        end

        drawnow;

      otherwise  % 'done'
        return
    end
end
