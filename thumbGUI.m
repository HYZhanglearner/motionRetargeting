function thumbGUI
% THUMBGUI Creates a GUI to control a 4-DOF thumb model and display a 3D finger curve.
%
%   This GUI allows independent control of:
%     1. CMC Bending
%     2. CMC Side
%     3. MCP Bending
%     4. IP Bending
%
%   The GUI displays:
%     - A 3D view of the thumb with a custom blue color and thick joint markers.
%     - A smooth 3D curve (in magenta) that is generated from three key joint
%       positions plus the fingertip.
%     - The fingertip is marked with a thick marker on the thumb display and in
%       a separate tip coordinate display.
%
%   This file requires buildThumb.m and computeFingerCurve.m to be in your MATLAB
%   path.
%
%   To run the GUI, simply type:
%       thumbGUI

%% Thumb segment lengths (units: e.g., mm)
L1 = 78.15;
L2 = 53.16;
L3 = 49.43;

%% Build the thumb robot model (from buildThumb.m)
thumb = buildThumb(L1, L2, L3);
% Initial configuration for the 4 DOFs: [CMC bending, CMC side, MCP bending, IP bending]
config = zeros(1,4);

%% Create the main figure window
fig = figure('Name','Thumb Control GUI','NumberTitle','off',...
             'Position',[100 100 1200 600]);

%% Create axes for robot display (left panel)
axRobot = subplot(1,2,1);
title(axRobot, 'Thumb Model & Finger Curve');
view(axRobot, 3);
axis(axRobot, 'equal');
grid(axRobot, 'on');

%% Create axes for tip coordinate display (right panel)
axTip = subplot(1,2,2);
hold(axTip, 'on');
grid(axTip, 'on');
xlabel(axTip, 'X');
ylabel(axTip, 'Y');
zlabel(axTip, 'Z');
title(axTip, 'Thumb Tip Position');
axis(axTip, [-150 150 -150 150 -150 150]);

% Plot the initial thumb tip position as a red dot (with a thick marker)
Ttip = getTransform(thumb, config, 'link3');
tipPos = tform2trvec(Ttip);
tipPlot = plot3(axTip, tipPos(1), tipPos(2), tipPos(3), ...
    'ro', 'MarkerSize', 16, 'MarkerFaceColor', 'r');

%% Create slider controls for each DOF
% Slider parameters
sliderWidth = 400;
sliderHeight = 20;
sliderX = 50;
sliderYStart = 50;
sliderYSpacing = 40;

% --- Slider for DOF 1: CMC Bending ---
slider1 = uicontrol('Style','slider',...
    'Min',-pi/2,'Max',pi/2,'Value',0,...
    'Position',[sliderX, sliderYStart + 3*sliderYSpacing, sliderWidth, sliderHeight],...
    'Callback',@updateRobot);
uicontrol('Style','text',...
    'Position',[sliderX + sliderWidth + 10, sliderYStart + 3*sliderYSpacing, 100, sliderHeight],...
    'String','CMC Bending');

% --- Slider for DOF 2: CMC Side ---
slider2 = uicontrol('Style','slider',...
    'Min',-pi/2,'Max',pi/2,'Value',0,...
    'Position',[sliderX, sliderYStart + 2*sliderYSpacing, sliderWidth, sliderHeight],...
    'Callback',@updateRobot);
uicontrol('Style','text',...
    'Position',[sliderX + sliderWidth + 10, sliderYStart + 2*sliderYSpacing, 100, sliderHeight],...
    'String','CMC Side');

% --- Slider for DOF 3: MCP Bending ---
slider3 = uicontrol('Style','slider',...
    'Min',-pi/2,'Max',pi/2,'Value',0,...
    'Position',[sliderX, sliderYStart + sliderYSpacing, sliderWidth, sliderHeight],...
    'Callback',@updateRobot);
uicontrol('Style','text',...
    'Position',[sliderX + sliderWidth + 10, sliderYStart + sliderYSpacing, 100, sliderHeight],...
    'String','MCP Bending');

% --- Slider for DOF 4: IP Bending ---
slider4 = uicontrol('Style','slider',...
    'Min',-pi/2,'Max',pi/2,'Value',0,...
    'Position',[sliderX, sliderYStart, sliderWidth, sliderHeight],...
    'Callback',@updateRobot);
uicontrol('Style','text',...
    'Position',[sliderX + sliderWidth + 10, sliderYStart, 100, sliderHeight],...
    'String','IP Bending');

%% Initial update to draw the robot, joint markers, and finger curve
updateRobot();

%% Callback function: updates the robot model, joint markers, finger curve, and tip display
    function updateRobot(~, ~)
        % Retrieve current angles from sliders
        theta1 = get(slider1, 'Value');
        theta2 = get(slider2, 'Value');
        theta3 = get(slider3, 'Value');
        theta4 = get(slider4, 'Value');
        % New configuration vector for the 4 DOFs
        config = [theta1, theta2, theta3, theta4];
        
        % --- Update the robot model display ---
        cla(axRobot);
        % Draw the thumb model in axRobot.
        show(thumb, config, 'Parent', axRobot, 'PreservePlot', false);
        
        % Find all patch objects (the surfaces) in axRobot and update their properties.
        patchHandles = findobj(axRobot, 'Type', 'Patch');
        for i = 1:length(patchHandles)
            set(patchHandles(i), 'FaceColor', [0.2 0.6 1], 'EdgeColor', 'none', 'LineWidth', 2);
        end
        
        % Reapply axes settings.
        view(axRobot, 3);
        axis(axRobot, 'equal');
        grid(axRobot, 'on');
        title(axRobot, 'Thumb Model & Finger Curve');
        hold(axRobot, 'on');
        
        % --- Get key joint positions for the finger curve ---
        % We choose the following bodies:
        %   'body1' - base (CMC bending),
        %   'body4' - MCP bending,
        %   'body6' - IP bending,
        %   'link3' - fingertip.
        jointNames = {'body1', 'body4', 'body6', 'link3'};
        curvePointsInput = zeros(length(jointNames), 3);
        for i = 1:length(jointNames)
            T = getTransform(thumb, config, jointNames{i});
            curvePointsInput(i,:) = tform2trvec(T);
            % Plot a thick marker at each joint.
            plot3(axRobot, curvePointsInput(i,1), curvePointsInput(i,2), curvePointsInput(i,3), ...
                'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
        end
        
        % --- Compute and plot the finger curve ---
        % The function computeFingerCurve returns a smooth set of points through the
        % provided joint positions.
        fingerCurve = computeFingerCurve(curvePointsInput, 100);
        plot3(axRobot, fingerCurve(:,1), fingerCurve(:,2), fingerCurve(:,3), 'm-', 'LineWidth', 3);
        
        % --- Plot the fingertip with a thicker marker on the robot display ---
        % Fingertip is the last point in curvePointsInput.
        plot3(axRobot, curvePointsInput(end,1), curvePointsInput(end,2), curvePointsInput(end,3), ...
            'ks', 'MarkerSize', 16, 'MarkerFaceColor', 'r');
        
        hold(axRobot, 'off');
        drawnow;
        
        % --- Update the tip position display (axTip) ---
        Ttip = getTransform(thumb, config, 'link3');
        tipPos = tform2trvec(Ttip);
        set(tipPlot, 'XData', tipPos(1), 'YData', tipPos(2), 'ZData', tipPos(3));
    end

end
