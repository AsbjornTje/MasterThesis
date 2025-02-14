function robot = createRobotModel_Corke(dhparams, jointTypes, q_home)
    % createRobotModel_Corke builds a SerialLink robot using the Corke Robotics Toolbox.
    % All rows in dhparams correspond to moving joints.
    %
    %   Inputs:
    %     dhparams  - an N-by-4 matrix with rows: [theta, d, a, alpha]
    %     jointTypes- a 1-by-N string array with entries "prismatic", "revolute", or "fixed"
    %     q_home    - a 1-by-N vector with the home configuration for the joints
    %
    %   For a prismatic joint, the joint variable is added to the constant d.
    %
    %   The robot’s base is rotated so that the prismatic joint’s default direction
    %   (its z-axis) aligns with the world-frame y-axis.
    
    n = size(dhparams, 1);
    L = [];  % initialize array of Links
    for i = 1:n
        theta_val = dhparams(i,1);
        d_val     = dhparams(i,2);
        a_val     = dhparams(i,3);
        alpha_val = dhparams(i,4);
        % Create the DH vector (order: [theta, d, a, alpha])
        dh_vec = [theta_val, d_val, a_val, alpha_val];
        if jointTypes(i) == "prismatic"
            Li = Link(dh_vec, 'sigma', 1);
        elseif jointTypes(i) == "fixed"
            Li = Link(dh_vec, 'qlim', [0 0]);
        else  % revolute
            Li = Link(dh_vec);
        end
        L = [L, Li];
    end
    
    robot = SerialLink(L, 'name', 'myRobot');
    % Set the base transform so that the default joint motion (along z) is rotated to world y.
    robot.base = trotx(-pi/2);
end
