
function collisionCylinders = generateCollisionEnvironment()
% Generates collision environment with 5 cylindrical objects
% Returns: collisionCylinders - Cell array of collision cylinders

collisionCylinders = {
    collisionCylinder(0.457, 8.0),  % Main Pipe
    collisionCylinder(0.044, 0.3),  % Front BLM (Small Goal Pipe)
    collisionCylinder(0.044, 0.3),  % Back BLM (Small Goal Pipe)
    collisionCylinder(0.350, 1.5),  % First Obstacle Pipe
    collisionCylinder(0.350, 1.5)   % Second Obstacle Pipe
};

cylinderPoses = [
    -0.958, 4.260, -1.707, 1, 0, 0, pi/2; 
    -0.458, 2.015, -1.627, 1, 0, 0, pi/2;
    -1.456, 6.921, -1.627, 1, 0, 0, pi/2; 
    -0.952, 5.400, -0.986, 1, 0, 0, 0;
    -0.952, 6.900, -0.986, 1, 0, 0, 0
];

for i = 1:length(collisionCylinders)
    P = trvec2tform(cylinderPoses(i, 1:3)); % Translation matrix
    R = axang2tform(cylinderPoses(i, 4:7)); % Rotation matrix
    collisionCylinders{i}.Pose = P * R; % Set pose
end

end
