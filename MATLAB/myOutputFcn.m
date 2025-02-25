function stop = myOutputFcn(results, state)
    % Define the threshold 
    threshold_obj = 500;  
    stop = false;
    if strcmp(state, 'iteration')
        % Check if the minimum objective value is below the threshold
        if results.MinObjective < threshold_obj
            stop = true;
            fprintf('Early termination: objective below %.4f.\n', threshold_obj);
        end
    end
end
