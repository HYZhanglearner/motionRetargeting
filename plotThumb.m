function plotThumb(ax, jointPositions)
% Plots thumb joints as points and connects them with lines

% Clear the existing plot
cla(ax);
hold(ax, 'on');

% Plot joint positions as black circles
plot3(ax, jointPositions(:,1), jointPositions(:,2), jointPositions(:,3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');

% Draw lines connecting the joints
plot3(ax, jointPositions(:,1), jointPositions(:,2), jointPositions(:,3), 'k-', 'LineWidth', 2);

% Set axis properties
grid(ax, 'on');
axis(ax, 'equal');
xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');
hold(ax, 'off');
end
