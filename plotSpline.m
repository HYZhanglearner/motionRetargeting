function plotSpline(ax, joints, color)
    % Fit and plot a smooth spline for the given joint positions
    t = linspace(0, 1, size(joints, 1));
    tt = linspace(0, 1, 50); % Higher resolution
    
    % Fit cubic spline
    splineX = spline(t, joints(:,1), tt);
    splineY = spline(t, joints(:,2), tt);
    splineZ = spline(t, joints(:,3), tt);
    
    % Plot the smoothed spline curve
    plot3(ax, splineX, splineY, splineZ, '-', 'Color', color, 'LineWidth', 2);
end
