function plotCylinder(cyl, T)
    % plotCylinder plots a cylinder (with semi-transparency) given its geometry and pose.
    %
    %   cyl is a structure with fields 'radius' and 'height'
    %   T is a 4x4 homogeneous transform specifying the pose.
    
    [X, Y, Z] = cylinder(cyl.radius, 20);  % 20 points around the circle
    Z = Z * cyl.height;
    % Transform the cylinder mesh points by T.
    pts = [X(:), Y(:), Z(:)]';
    pts_trans = T(1:3,1:3)*pts + T(1:3,4);
    X_t = reshape(pts_trans(1,:), size(X));
    Y_t = reshape(pts_trans(2,:), size(Y));
    Z_t = reshape(pts_trans(3,:), size(Z));
    surf(X_t, Y_t, Z_t, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
end