function exampleHelperConstrainedRobotMoveGripper(kinova, endConfig, startJointVal, endJointVal)
%exampleHelperConstrainedRobotMoveGripper Visualize and move the robot gripper
numInterpolations = 10;
for i = 1:numInterpolations
    jointval = startJointVal+(endJointVal-startJointVal)/numInterpolations*i;
    config = endConfig;
    config(end-1:end) = jointval;
    show(kinova, config, ...
        "Visuals", "on", ...
        "FastUpdate",true, ...
        "Frames", "off", ...
        "PreservePlot",false);
    drawnow;
end
end