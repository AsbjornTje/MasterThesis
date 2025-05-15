function plotFullManipEllipsoids(robot, goalPoses, collisionCylinders)
  ee = robot.BodyNames{end};
  gik = generalizedInverseKinematics('RigidBodyTree',robot,...
                                     'ConstraintInputs',{'pose'});
  q0 = homeConfiguration(robot);
  cmap = lines(size(goalPoses,1));

  figure('Name','Trans + Rot Manipulability','NumberTitle','off');
  tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

  % --- Translational ellipsoids ---
  ax1 = nexttile;
  hold(ax1,'on'); view(ax1,3); grid(ax1,'on'); axis(ax1,'equal');
  title(ax1,'Translational Manipulability');
  xlabel(ax1,'X'); ylabel(ax1,'Y'); zlabel(ax1,'Z');
  for i=1:size(goalPoses,1)
    % IK solve as before...
    pos = goalPoses(i,1:3);
    axang = goalPoses(i,4:7);
    poseTgt = constraintPoseTarget(ee);
    poseTgt.TargetTransform = trvec2tform(pos)*axang2tform(axang);
    [q_sol,info] = gik(q0,poseTgt);
    if info.Status~="success", continue; end
    q0 = q_sol;

    J = geometricJacobian(robot,q_sol,ee);
    Jv = J(1:3,:);          % translational part
    [Vt,Dt] = eig(Jv*Jv');
    rt = sqrt(diag(Dt));
    [x,y,z] = ellipsoid(0,0,0, rt(1),rt(2),rt(3),20);
    pts = [x(:),y(:),z(:)]*Vt';
    X = reshape(pts(:,1),size(x)) + pos(1);
    Y = reshape(pts(:,2),size(y)) + pos(2);
    Z = reshape(pts(:,3),size(z)) + pos(3);
    surf(X,Y,Z,'FaceAlpha',0.3,'EdgeColor','none','FaceColor',cmap(i,:));
  end

  % --- Rotational ellipsoids or axes ---
  ax2 = nexttile;
  hold(ax2,'on'); view(ax2,3); grid(ax2,'on'); axis(ax2,'equal');
  title(ax2,'Rotational Manipulability');
  for i=1:size(goalPoses,1)
    % reuse the same q_sol from above (or resolve IK)
    pos = goalPoses(i,1:3);
    J = geometricJacobian(robot,q_sol,ee);
    Jw = J(4:6,:);         % rotational part
    [Vr,Dr] = eig(Jw*Jw');
    rr = sqrt(diag(Dr));
    % draw the three principal axes as arrows
    for k=1:3
      dir = Vr(:,k);
      quiver3(pos(1),pos(2),pos(3), ...
              dir(1)*rr(k),dir(2)*rr(k),dir(3)*rr(k), ...
              'LineWidth',2,'Color',cmap(i,:), ...
              'MaxHeadSize',0.5);
    end
  end

  % overlay cylinders on both axes
  for ax = [ax1 ax2]
    axes(ax);
    for c = 1:numel(collisionCylinders)
      show(collisionCylinders{c},'Parent',ax);
    end
  end
end
