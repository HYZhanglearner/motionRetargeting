function q_robot = retargetThumb(humanJoints, robotThumb, humanSplineLength)
% Retargets human thumb motion to robot thumb using dynamic scaling and IK

% Compute robot spline arc length using default (zero) configuration
robotJointsInit = getRobotThumbPositions(zeros(1, 4), robotThumb);
robotSplineLength = sum(vecnorm(diff(robotJointsInit([2,3,6], :)), 2, 2));

% Compute scale factor dynamically
scaleFactor = humanSplineLength / robotSplineLength;

% Scale human positions to match robot proportions
robotTargetJoints = humanJoints * scaleFactor;

% Solve inverse kinematics
ik = inverseKinematics('RigidBodyTree', robotThumb);
weights = [1, 1, 1, 0.1, 0.1, 0.1];

% Solve for robot J1–J4
q_robot = zeros(1, 4);
for i = 1:4
    tform = trvec2tform(robotTargetJoints(i, :));
    [qSol, ~] = ik('thumb_fingertip', tform, weights, zeros(1,4));
    q_robot(i) = qSol(i);
end
end
