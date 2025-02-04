classdef exampleHelperConstrainedStateSpace < manipulatorStateSpace
    properties
        %Region Constrained Region of the end-effector
        Region

        %Robot rigidBodyTree object of the state space
        Robot

        %IKSolver Inverse Kinematics solver to get a constrained joint configuration
        IKSolver

        %EnableConstraint Logical value that indicates if the constraint is enabled
        EnableConstraint
    end

    methods
        function obj = exampleHelperConstrainedStateSpace(rbt, endEffectorName, targetOrientation)
            %Constructor
            obj@manipulatorStateSpace(rbt);
            obj.EnableConstraint = true;
            obj.Region = workspaceGoalRegion(endEffectorName, 'EndEffectorOffsetPose', targetOrientation);
            obj.Region.Bounds = [-100, 100; -100 100; -100 100; -pi, pi; 0, 0; 0, 0];
            
            % Store a reference of the manipulatorStateSpace/RigidBodyTree
            % in the Robot property. Note that the RigidBodyTree property
            % of the manipulatorStateSpace is read-only. Hence, accessing
            % it will involve creating a copy of the underlying handle,
            % which can be expensive.
            obj.Robot = obj.RigidBodyTree;
            
            % Configure the IK solver
            obj.IKSolver = inverseKinematics('RigidBodyTree', obj.Robot);
            obj.IKSolver.SolverParameters.AllowRandomRestart = false;
        end

        function constrainedStates = interpolate(obj, state1, state2, ratios)
            constrainedStates = interpolate@manipulatorStateSpace(obj, state1, state2, ratios);
            if(obj.EnableConstraint)
                for i = 1:size(constrainedStates, 1)
                    constrainedStates(i,:) = obj.constrainConfig(constrainedStates(i,:));
                end
            end
        end

        function config = jointConfigurationGiven(obj, eePose)
            %jointConfigurationGiven Outputs the joint configuration given an end-effector pose
            config = obj.IKSolver(obj.Region.EndEffectorName, ...
                eePose,  ...
                ones(1, 6), ...
                obj.Robot.homeConfiguration());
        end
    end

    methods(Access=private)
        function constrainedConfig = constrainConfig(obj, config)
            %constrainConfig Constraint joint configuration to the region
            %   The function finds the joint configuration corresponding to
            %   the end-effector pose closest to the constrained region.
            wgr = obj.Region;
            T0_s = obj.Robot.getTransform(config, wgr.EndEffectorName);
            T0_w = wgr.ReferencePose;
            Tw_e = wgr.EndEffectorOffsetPose;
            Tw_sPrime = T0_w \ T0_s / Tw_e;
            dw = convertTransformToPoseVector(obj, Tw_sPrime);
            bounds = wgr.Bounds;
            
            % Find the pose vector closeset to the bounds of the region.
            for dofIdx = 1:6
                if(dw(dofIdx) < bounds(dofIdx, 1))
                    dw(dofIdx) = bounds(dofIdx, 1);
                elseif(dw(dofIdx) > bounds(dofIdx, 2))
                    dw(dofIdx) = bounds(dofIdx, 2);
                end
            end
            
            % Convert the pose vector in the region's reference frame to a
            % homogeneous transform.
            constrainedPose = obj.convertPoseVectorToTransform(dw);
            
            % Convert this pose to world coordinates, and find the
            % corresponding joint configuration.
            constrainedPose = T0_w * constrainedPose * Tw_e;
            constrainedConfig = obj.IKSolver(obj.Region.EndEffectorName, ...
                constrainedPose,  ...
                ones(1, 6), ...
                config);
            constrainedConfig=avoidJointLimitsWarning(obj,constrainedConfig);
        end

        function qout=avoidJointLimitsWarning(obj,qin)
            %avoidJointLimitsWarning Avoid joint limits warning
            %   When the constrained config joint values are really close
            %   to the joint limits, and are used as inputs to the solver,
            %   it is possible that they might be outside the joint limits,
            %   and will cause the solver to emit a warning.
            qout=qin;
            tol=sqrt(eps);
            relevantJointConfig=qin(1:end-2);
            upperlim=obj.StateBounds(1:end-2,2)';
            lowerlim=obj.StateBounds(1:end-2,1)';
            absDiffUpper=abs(relevantJointConfig-upperlim);
            absDiffLower=abs(relevantJointConfig-lowerlim);
            isCloseToUpperLimit=absDiffUpper<tol;
            isCloseToLowerLimit=absDiffLower<tol;

            % For a joint value closer to the upper limit, set it to the
            % upper limit reduced by some tolerance. For joint values
            % closer to the lower limit, set it to the lower limit
            % incremented by some tolerance.
            qout(isCloseToUpperLimit)=upperlim(isCloseToUpperLimit)-tol;
            qout(isCloseToLowerLimit)=lowerlim(isCloseToLowerLimit)+tol;
        end

        function poseVector = convertTransformToPoseVector(~, tform)
            %convertTransformToPoseVector Transformation matrix to pose vector
            %   A pose vector is a 6-element vector with first three
            %   elements corresponding to the translation component and the
            %   next three elements are euler angles of the form "ZYX".
            poseVector = zeros(1, 6);
            poseVector(1:3) = tform2trvec(tform);
            poseVector(4:end) = tform2eul(tform, "ZYX");
        end

        function tform = convertPoseVectorToTransform(~, poseVector)
            %convertPoseVectorToTransform Pose vector to a transformation matrix
            %   A pose vector is a 6-element vector with first three
            %   elements corresponding to the translation component and the
            %   next three elements are euler angles of the form "ZYX".
            tform = eul2tform(poseVector(4:end));
            tform(1:3, end) = poseVector(1:3)';
        end
    end
end