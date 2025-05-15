function stop = plotDesignVsFoldability(results,state)
% plotDesignVsFoldability   Scatter of design‐sum vs foldability over iter
%   stop = plotDesignVsFoldability(results,state)
%   • results : BayesianOptimization object
%   • state   : 'initial','iteration','done'

    stop = false;
    persistent figDVF
    figName = 'Design Sum vs Foldability';

    switch state
      case 'initial'
        % create the window and return
        figDVF = figure('Name',figName,'NumberTitle','off');
        return

      case 'iteration'
        % bring up & clear
        if isempty(figDVF) || ~ishandle(figDVF)
            figDVF = figure('Name',figName,'NumberTitle','off');
        else
            figure(figDVF); clf;
        end

        T = results.XTrace;
        Y = results.ObjectiveTrace;
        if isempty(T) || isempty(Y)
            return
        end

        % compute design sum = |d8|+|d10|+|d12|
        ds = abs(T.d8) + abs(T.d10) + abs(T.d12);

        % recover foldability ≃ objective – designSum
        fold = Y - ds;

        % scatter
        scatter(ds, fold, 50, 'filled');
        xlabel('Design Sum = |d8|+|d10|+|d12|','FontSize',12);
        ylabel('Foldability = Obj – DesignSum','FontSize',12);
        title(figName,'FontSize',14);
        grid on;
        drawnow;

      otherwise  % 'done'
        return
    end
end
