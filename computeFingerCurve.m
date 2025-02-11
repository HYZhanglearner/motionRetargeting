function curvePoints = computeFingerCurve(jointPoints, numPoints)
% COMPUTEFINGERCURVE Computes a smooth 3D curve that interpolates the given points.
%
%   curvePoints = computeFingerCurve(jointPoints, numPoints) returns a 
%   numPoints-by-3 matrix of [x, y, z] coordinates representing a smooth curve 
%   that passes through the input jointPoints. The input jointPoints should be
%   an N-by-3 matrix (for example, positions from N joints along the finger).
%
%   The function uses cumulative chord length parameterization and spline 
%   interpolation to generate the smooth curve.
%
%   Example:
%      points = [0 0 0; 10 5 0; 20 10 5; 30 15 10];
%      curve = computeFingerCurve(points, 100);
%      plot3(curve(:,1), curve(:,2), curve(:,3), 'r-', 'LineWidth',2);

if nargin < 2
    numPoints = 100;
end

% Compute cumulative chord lengths for the parameterization.
dists = [0; cumsum(sqrt(sum(diff(jointPoints,1,1).^2, 2)))];
t = dists;  % Parameter values

% Create a dense set of parameter values.
t_dense = linspace(t(1), t(end), numPoints);

% Spline interpolation for each coordinate.
x_dense = spline(t, jointPoints(:,1), t_dense);
y_dense = spline(t, jointPoints(:,2), t_dense);
z_dense = spline(t, jointPoints(:,3), t_dense);

curvePoints = [x_dense(:), y_dense(:), z_dense(:)];
end
