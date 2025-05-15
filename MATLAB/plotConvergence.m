function stop = plotConvergence(results,state)
% plotD10D12  Custom BayesOpt plot: d10 on x‐axis vs d12 on y‐axis
%
%   stop = plotD10D12(results,state) is called by bayesopt at each stage.
%   results is a BayesianOptimization object, state is 'initial' / 'iteration' / 'done'.

    stop = false;   % never halt the optimizer

    switch state
        case 'initial'
            % set up the figure once
            figure('Name','Convergence','NumberTitle','off');
            hold on;
            xlabel('d10');
            ylabel('d12');
            title('BayesOpt Convergence');

        case 'iteration'
            % pull out the history of evaluated points
            T = results.XTrace;  %# XTrace is a table of dimension [numIter×numVars]
            if all(ismember({'d10','d12'}, T.Properties.VariableNames))
                d10 = T.d10;
                d12 = T.d12;
                cla;                         % clear previous scatter
                scatter(d10, d12, 20, 'k', 'filled');% plot all evaluated points
                drawnow;                     % force redraw
            end

        case 'done'
            hold off;
    end
end
