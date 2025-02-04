% Clear workspace
clear; clc;

% Define the Denavit-Hartenberg (DH) parameters for a 4-DOF robot
syms theta1 theta2 theta3 theta4 L1 L2 L3 L4 real

% DH Table [theta d a alpha]
DH_params = [
    theta1  0  L1  pi/2;
    theta2  0  L2  0;
    theta3  0  L3  0;
    theta4  0  L4  0;
];

% Compute the transformation matrices
T01 = dh_transform(DH_params(1,:));
T12 = dh_transform(DH_params(2,:));
T23 = dh_transform(DH_params(3,:));
T34 = dh_transform(DH_params(4,:));

% Compute forward kinematics
T04 = simplify(T01 * T12 * T23 * T34); % End-effector pose


%% 

% Objective Function: Minimize total link length and number of joints
objFunc = @(x) sum(x(1:4)) + 0.1 * sum(x(5:8)); % x(1:4) are link lengths, x(5:8) are joint angles

% Constraints
goal_poses = [0.5, 0.3, 1.0; -0.3, 0.4, 0.8; 0.2, -0.5, 1.2]; % Target end-effector positions

% Constraint function
constraintFunc = @(x) reachability_constraint(x, goal_poses);

% Bounds
lb = [0.2, 0.2, 0.2, 0.2, -pi, -pi, -pi, -pi]; % Min link lengths & angles
ub = [1.0, 1.0, 1.0, 1.0, pi, pi, pi, pi]; % Max link lengths & angles

% Solve optimization using Genetic Algorithm
options = optimoptions('ga', 'Display', 'iter', 'PopulationSize', 100);
[x_opt, fval] = ga(objFunc, 8, [], [], [], [], lb, ub, constraintFunc, options);

% Extract optimized link lengths and joint angles
L_opt = x_opt(1:4);
theta_opt = x_opt(5:8);


%% 


function [c, ceq] = reachability_constraint(x, goal_poses)
    L = x(1:4); % Extract link lengths
    theta = x(5:8); % Extract joint angles

    % Compute forward kinematics using optimized values
    T04 = forward_kinematics(L, theta);
    
    % Compute error for each target pose
    c = [];
    for i = 1:size(goal_poses, 1)
        error = norm(T04(1:3,4) - goal_poses(i,:)'); % Position error
        c = [c; error - 0.01]; % Constraint: Position error should be ≤ 0.01m
    end
    
    ceq = [];
end


%% 

% Visualization
figure;
hold on;
xlabel('X-axis'); ylabel('Y-axis'); zlabel('Z-axis');
grid on;
axis([-1 1 -1 1 0 1.5]);

% Plot base
plot3(0, 0, 0, 'ko', 'MarkerSize', 10, 'LineWidth', 2);

% Compute transformation matrices for visualization
T01 = dh_transform([theta_opt(1), 0, L_opt(1), pi/2]);
T12 = dh_transform([theta_opt(2), 0, L_opt(2), 0]);
T23 = dh_transform([theta_opt(3), 0, L_opt(3), 0]);
T34 = dh_transform([theta_opt(4), 0, L_opt(4), 0]);

% Compute joint positions
p0 = [0; 0; 0];
p1 = T01(1:3,4);
T02 = T01 * T12;
T03 = T02 * T23;
T04 = T03 * T34;

p2 = T02(1:3,4); % Extract position from transformation matrix
p3 = T03(1:3,4);
p4 = T04(1:3,4);


% Plot links
plot3([p0(1), p1(1)], [p0(2), p1(2)], [p0(3), p1(3)], 'r-', 'LineWidth', 2);
plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'g-', 'LineWidth', 2);
plot3([p2(1), p3(1)], [p2(2), p3(2)], [p2(3), p3(3)], 'b-', 'LineWidth', 2);
plot3([p3(1), p4(1)], [p3(2), p4(2)], [p3(3), p4(3)], 'k-', 'LineWidth', 2);

% Plot end-effector position
plot3(p4(1), p4(2), p4(3), 'bo', 'MarkerSize', 8, 'LineWidth', 2);

title('Optimized 4-DOF Manipulator');
hold off;

