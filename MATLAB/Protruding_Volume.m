function [volX, volY, volZ] = Protruding_Volume(robot, collisionData, q_config, goalRegion, numSamples, debugFlag)
% Protruding_Volume estimates the volume of the robot's collision cylinders 
% that lie outside the goal region along the x, y, and z directions.
%
% Only the collision data for the last 6 bodies are processed.
%
% Inputs:
%   robot        - A rigidBodyTree robot model.
%   collisionData- A structure with fields corresponding to body names; each
%                  field is a cell array of collision data structures (fields:
%                  Radius, Length, Pose).
%   q_config     - The robot configuration (e.g., folded configuration).
%   goalRegion   - A structure with:
%                     .ReferencePose: 4x4 transform defining the goal frame.
%                     .Bounds: 3x2 matrix; each row is [min, max] for x, y, and z.
%   numSamples   - (Optional) Number of random sample points per collision object (default: 1000).
%   debugFlag    - (Optional) If true, a debug plot is shown.
%
% Outputs:
%   volX, volY, volZ - Estimated volumes (in cubic units) of collision geometry 
%                      that lie outside the goal region along x, y, and z.

if nargin < 5
    numSamples = 1000;
end
if nargin < 6
    debugFlag = false;
end

volX = 0;
volY = 0;
volZ = 0;
allPts = [];

totalBodies = robot.NumBodies;
minBodyIndex = totalBodies - 6 + 1;  % Process only the last 6 bodies.

bodyNames = fieldnames(collisionData);
for i = 1:length(bodyNames)
    bodyName = bodyNames{i};
    % Extract numeric part from body name (assumes format 'bodyN')
    numStr = regexp(bodyName, '\d+', 'match');
    if isempty(numStr)
        continue;
    end
    bodyIndex = str2double(numStr{1});
    if bodyIndex < minBodyIndex
        continue;
    end
    
    dataCell = collisionData.(bodyName);
    T_body = getTransform(robot, q_config, bodyName); % Current global transform of the body.
    for j = 1:length(dataCell)
        data = dataCell{j};
        r = data.Radius;
        h = data.Length;
        cylVolume = pi * r^2 * h;
        sampleWeight = cylVolume / numSamples;
        
        % Use the stored Pose (which includes the necessary shift).
        T_cyl = T_body * data.Pose;
        
        % Sample points uniformly within the cylinder (local frame: base at z=0, top at z=h).
        pts_local = sampleCylinderPoints(r, h, numSamples);
        % Transform sample points to global coordinates.
        pts_global = transformPointsForwardCustom(T_cyl, pts_local);
        allPts = [allPts; pts_global];
        
        % Extract coordinates.
        x = pts_global(:,1);
        y = pts_global(:,2);
        z = pts_global(:,3);
        
        % Compute global goal region box by transforming its local vertices.
        [X, Y, Z] = ndgrid([goalRegion.Bounds(1,1), goalRegion.Bounds(1,2)], ...
                            [goalRegion.Bounds(2,1), goalRegion.Bounds(2,2)], ...
                            [goalRegion.Bounds(3,1), goalRegion.Bounds(3,2)]);
        vertices_local = [X(:), Y(:), Z(:), ones(numel(X),1)];
        vertices_world = (goalRegion.ReferencePose * vertices_local')';
        vertices_world = vertices_world(:,1:3);
        global_xMin = min(vertices_world(:,1));
        global_xMax = max(vertices_world(:,1));
        global_yMin = min(vertices_world(:,2));
        global_yMax = max(vertices_world(:,2));
        global_zMin = min(vertices_world(:,3));
        global_zMax = max(vertices_world(:,3));
        
        % Determine which sample points are outside along each axis.
        oX = (x < global_xMin) | (x > global_xMax);
        oY = (y < global_yMin) | (y > global_yMax);
        oZ = (z < global_zMin) | (z > global_zMax);
        
        volX = volX + sum(sampleWeight * oX);
        volY = volY + sum(sampleWeight * oY);
        volZ = volZ + sum(sampleWeight * oZ);
    end
end

if debugFlag && ~isempty(allPts)
    figure('Position',[100, 100, 1200, 800]);
    scatter3(allPts(:,1), allPts(:,2), allPts(:,3), 5, 'b', 'filled');
    hold on;
    % Plot the global goal region box.
    K = convhull(vertices_world);
    trisurf(K, vertices_world(:,1), vertices_world(:,2), vertices_world(:,3), 'FaceAlpha', 0.3, 'FaceColor', 'r');
    % Also show the robot stick figure.
    show(robot, q_config, "Visuals", "off", "Collisions", "off", "PreservePlot",false);
    title('Sample Points in World Frame & Global Goal Region Box');
    xlabel('X'); ylabel('Y'); zlabel('Z'); grid on; axis equal;
    view(210,20);
    disp('Press any key to start path planning...');
    pause;
    pause(0.2);
    hold off;
end

end

function pts = sampleCylinderPoints(r, h, numSamples)
theta = 2*pi*rand(numSamples, 1);
rho = r * sqrt(rand(numSamples, 1));
x = rho .* cos(theta);
y = rho .* sin(theta);
z = h * rand(numSamples, 1);
pts = [x, y, z];
end

function ptsOut = transformPointsForwardCustom(T, pts)
N = size(pts,1);
ptsH = [pts, ones(N,1)];
ptsT = (T * ptsH')';
ptsOut = ptsT(:,1:3);
end
