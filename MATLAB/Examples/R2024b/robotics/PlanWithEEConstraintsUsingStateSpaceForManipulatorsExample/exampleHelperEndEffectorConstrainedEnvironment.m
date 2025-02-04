function [robot, env] = exampleHelperEndEffectorConstrainedEnvironment()
% exampleHelperEndEffectorConstrainedEnvironment
% Helper function to load the Kinova Gen 3 robot and a simple 
% environment of collision objects placed in the robot model world.
%
% This function is for example purposes and may be removed in a future
% release.
% Copyright 2021 The Mathworks, Inc. 

% Create environment as a set of collision objects.
bench = collisionBox(0.5, 0.9, 0.05);
belt1 = collisionBox(1.3, 0.4, 0.235);
barricade = collisionBox(1.6, 0.03, 0.35);

TBench = trvec2tform([0.35 0 0.1]);
TBelt1 = trvec2tform([0 -0.6 0.1]);

bench.Pose = TBench;
belt1.Pose = TBelt1;
barricade.Pose = trvec2tform([0.3, -0.25, 0.3]);
cylinder1 = collisionCylinder(0.03, 0.1);
cylinder2 = collisionCylinder(0.03, 0.1);
cylinder3 = collisionCylinder(0.03, 0.1);

TCyl = trvec2tform([0.5 0.15 0.178]);
TCyl2 = trvec2tform([0.52 0 0.178]);
TCyl3 = trvec2tform([0.4 -0.1 0.18]);

cylinder1.Pose = TCyl;
cylinder2.Pose = TCyl2;
cylinder3.Pose = TCyl3;
env = {bench, belt1, cylinder1, cylinder2, cylinder3, barricade};
robot = loadrobot("kinovaGen3", "DataFormat", "row");
robot.Bodies{end}.addVisual('mesh', 'exampleHelperBase_EELink.stl', trvec2tform([0, 0, -0.02]))
finger_1 = rigidBody("finger_1");
finger_1.addVisual('mesh', 'examlpeHelperFinger_EELink.stl', eul2tform([0 pi/2 0])* trvec2tform([0 0.00 -0.03]));
finger_1.Joint = rigidBodyJoint("finger_1_ee", "prismatic");
finger_1.Joint.setFixedTransform(eul2tform([pi/2 -pi/2 0], 'XYZ'))
finger_2 = rigidBody("finger_2");
finger_2.addVisual('mesh', 'examlpeHelperFinger_EELink.stl', eul2tform([0 pi/2 0])* trvec2tform([0 0.00 -0.03]));
finger_2.Joint = rigidBodyJoint("finger_2_ee", "prismatic");
finger_2.Joint.setFixedTransform(eul2tform([pi/2 pi/2 0], 'XYZ'))
robot.addBody(finger_1, 'EndEffector_Link')
robot.addBody(finger_2, 'EndEffector_Link')
end