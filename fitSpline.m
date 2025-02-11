function splineCurve = fitSpline(jointPositions)
% Fits a cubic spline through given joint positions

t = linspace(0, 1, size(jointPositions, 1));
tq = linspace(0, 1, 50); % Dense query points

splineX = spline(t, jointPositions(:,1), tq);
splineY = spline(t, jointPositions(:,2), tq);
splineZ = spline(t, jointPositions(:,3), tq);

splineCurve = [splineX', splineY', splineZ'];
end
