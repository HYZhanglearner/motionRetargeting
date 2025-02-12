function q_robot = retargetThumb(humanJoints, robotThumb, mapping)
% RETARGETTHUMB retargets human thumb motion to the robot thumb.
%
% Inputs:
%   humanJoints - an N×3 matrix of human joint positions.
%   robotThumb  - a robotics.RigidBodyTree model of the robot thumb.
%   mapping     - a structure with fields:
%                   mapping.humanIndices: indices (any length) to pick from humanJoints.
%                   mapping.robotIndices: indices (any length) to pick from robot joints.
%                   mapping.targetLink  : (optional) name of the robot target link for IK.
%
% Output:
%   q_robot - 1×4 vector of robot joint angles.
%
% This function computes a scale factor based on the arc lengths of the
% control curves and then uses an optimization routine (fmincon) with a
% cost function that compares the resampled curves.
    % Use a persistent variable to store the previous solution
    persistent q_prev;
    if isempty(q_prev)
        q_init = [0.1, 0.1, 0.1, 0.1];  % Default starting configuration if none exists
    else
        q_init = q_prev;       % Use the previous solution as the initial guess
    end

    % --- Extract control points (flexible lengths) ---
    humanControlPoints = humanJoints(mapping.humanIndices, :);
    
    % --- Compute arc lengths for scaling ---
    humanSplineLength = computeSplineLength(humanControlPoints);
    
    % Get default robot joint positions (e.g., zero configuration).
    q0 = zeros(1, 4);
    robotJointsInit = getRobotThumbPositions(q0, robotThumb);
    robotControlPointsInit = robotJointsInit(mapping.robotIndices, :);
    robotSplineLength = computeSplineLength(robotControlPointsInit);
    
    % --- Compute scale factor ---
    scaleFactor = humanSplineLength / robotSplineLength;
    
    % Scale the human control points to obtain the target robot curve.
    targetHumanPoints = humanControlPoints * scaleFactor;
    
    % --- Define the cost function ---
    costFunc = @(q) robotControlCost(q, targetHumanPoints, robotThumb, mapping);
    
    % --- Optimize to find the robot joint angles ---
    lb = -pi * ones(1, 4);
    ub =  pi * ones(1, 4);
    options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp');
    q_robot = fmincon(costFunc, q_init, [], [], [], [], lb, ub, [], options);
    
    q_prev = q_robot;
end

%% Helper Function: Compute Spline Arc Length
function L = computeSplineLength(controlPoints)
    % Resample the spline with a fixed number of sample points.
    numSamples = 50;
    splinePoints = sampleSpline(controlPoints, numSamples);
    ds = diff(splinePoints);
    L = sum(vecnorm(ds, 2, 2));
end

%% Helper Function: Sample a Spline from Control Points
function splinePoints = sampleSpline(controlPoints, numSamples)
    % Build a cubic spline that interpolates the provided control points.
    % The original parameter is defined based on the number of points.
    t = linspace(0, 1, size(controlPoints, 1));
    tt = linspace(0, 1, numSamples);
    splineX = spline(t, controlPoints(:, 1), tt);
    splineY = spline(t, controlPoints(:, 2), tt);
    splineZ = spline(t, controlPoints(:, 3), tt);
    splinePoints = [splineX(:), splineY(:), splineZ(:)];
end

%% Helper Function: Cost Function for Curve Matching
function cost = robotControlCost(q, targetPoints, robotThumb, mapping)
    % For a given robot configuration q, compute its joint positions.
    robotJoints = getRobotThumbPositions(q, robotThumb);
    % Extract the robot control points (of flexible length).
    robotControlPoints = robotJoints(mapping.robotIndices, :);
    
    % Resample both the robot and target (scaled human) curves.
    numSamples = 50;
    robotCurve = sampleSpline(robotControlPoints, numSamples);
    targetCurve = sampleSpline(targetPoints, numSamples);
    
    % Compute the cost as the sum of squared distances between the curves.
    cost = sum(vecnorm(robotCurve - targetCurve, 2, 2).^2);
end
