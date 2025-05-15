function S = computeLocalSensitivities(bestParams, dhFull, jointTypes, goalRegion, q_home)
% S is 3×7 sensitivities [dVolX/dp; dVolY/dp; dVolZ/dp] for each DH parameter

    vars = {'a6','d7','d8','d9','d10','d11','d12'};
    n    = numel(vars);
    epsv = 1e-3;

    % Baseline volumes
    dhBase = applyParamsToDH(dhFull, bestParams);
    [VX0, VY0, VZ0] = computeProtrusionComponents(dhBase, jointTypes, goalRegion, q_home);

    S = zeros(3,n);
    for k = 1:n
        % bump up
        bp = bestParams; bp.(vars{k}) = bp.(vars{k}) + epsv;
        dhp = applyParamsToDH(dhFull, bp);
        [VXp, VYp, VZp] = computeProtrusionComponents(dhp, jointTypes, goalRegion, q_home);

        % bump down
        bm = bestParams; bm.(vars{k}) = bm.(vars{k}) - epsv;
        dhm = applyParamsToDH(dhFull, bm);
        [VXm, VYm, VZm] = computeProtrusionComponents(dhm, jointTypes, goalRegion, q_home);

        % central difference
        S(1,k) = (VXp - VXm)/(2*epsv);
        S(2,k) = (VYp - VYm)/(2*epsv);
        S(3,k) = (VZp - VZm)/(2*epsv);
    end
end

function dh = applyParamsToDH(dhFull, T)
    % copy DH and overwrite the seven design entries
    dh = dhFull;
    dh(6,1)  = T.a6;
    dh(7,3)  = T.d7;
    dh(8,3)  = T.d8;
    dh(9,3)  = T.d9;
    dh(10,3) = T.d10;
    dh(11,3) = T.d11;
    dh(12,3) = T.d12;
end

function [vx_min, vy_min, vz_min] = computeProtrusionComponents(dh, jointTypes, goalRegion, q_home)
    % rebuild robot & skiplist
    [robot, collisionData] = createRobotCollisionModel(dh, jointTypes, q_home);
    adj = [robot.Base.Name, robot.BodyNames];
    skip = cell(robot.NumBodies,2);
    for m = 1:robot.NumBodies
        skip(m,:) = {adj{m}, adj{m+1}};
    end
    skip(end+1,:) = {'body6','body8'};

    % fixed “fold” pose and roll‐joint indices
    q_fold = [0 pi/2 pi/2 -0.7 0 pi 0 pi 0];
    idx8   = 5; idx10 = 7;
    step   = deg2rad(30);

    vx_min = inf; vy_min = inf; vz_min = inf;
    for r8 = -pi:step:pi
      for r10 = -pi:step:pi
        q = q_fold;
        q(idx8)  = r8;
        q(idx10) = r10;
        [vX,vY,vZ] = Protruding_Volume(robot,collisionData,q,goalRegion,1000,false);
        vx_min = min(vx_min, vX);
        vy_min = min(vy_min, vY);
        vz_min = min(vz_min, vZ);
      end
    end
end
