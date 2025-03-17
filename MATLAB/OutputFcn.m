function stop = OutputFcn(results, state)
   stop = false;  % By default, do not stop the optimization

switch state
    case 'init'
        % Create a new figure for this plot
        figure('Name','myLogMinObjective','NumberTitle','off');
        % Use a log-scale on the y-axis
        set(gca,'YScale','log');
        hold on
        xlabel('Function Evaluations');
        ylabel('Objective (log scale)');
        title('Minimum Observed and Estimated Objective (log scale)');
        
    case 'iteration'
        % Current iteration number
        it = results.Iteration;
        
        % Observed objective values so far
        allObjValues = results.ObjectiveTrace(1:it);
        % Minimum observed objective at each iteration
        minObsSoFar = cummin(allObjValues);
        
        % Estimated min objective at each iteration
        estMinObj = results.EstimatedObjectiveTrace(1:it);
        
        % Clear and re-plot
        cla
        plot(1:it, minObsSoFar, 'b-', 'LineWidth', 2);
        hold on
        plot(1:it, estMinObj, 'g-', 'LineWidth', 2);
        legend('Min Observed Objective','Estimated Min Objective','Location','best');
        drawnow
        
    case 'done'
        % Optional: finalize or annotate the figure
end
end