function thumbXacroGUI
% GUI for controlling and retargeting human thumb to robot thumb

%% Build Models
humanThumb = buildThumb();
robotThumb = buildThumbXacro();

%% Create GUI
fig = figure('Name', 'Thumb Control GUI', 'NumberTitle', 'off', ...
             'Position', [100, 100, 1300, 600]);

% Axes
axHuman = subplot(1,2,1); title(axHuman, 'Human Thumb'); view(axHuman, 3); hold(axHuman, 'on');
axRobot = subplot(1,2,2); title(axRobot, 'Robot Thumb'); view(axRobot, 3); hold(axRobot, 'on');

%% Sliders for Human Thumb
for i = 1:4
    slidersHuman(i) = uicontrol('Style', 'slider', 'Min', -pi/2, 'Max', pi/2, 'Value', 0, ...
        'Units', 'normalized', 'Position', [0.1, 0.2 - (i-1)*0.04, 0.3, 0.03], ...
        'Callback', @updateThumb);
end

%% Update Function
function updateThumb(~,~)
    q_human = [get(slidersHuman(1), 'Value'), get(slidersHuman(2), 'Value'), ...
               get(slidersHuman(3), 'Value'), get(slidersHuman(4), 'Value')];

    humanJoints = getHumanThumbPositions(q_human, humanThumb);
    q_robot = retargetThumb(humanJoints, robotThumb, sum(vecnorm(diff(humanJoints), 2, 2)));
    robotJoints = getRobotThumbPositions(q_robot, robotThumb);

    plotThumb(axHuman, humanJoints);
    plotThumb(axRobot, robotJoints);
end

updateThumb();
end
