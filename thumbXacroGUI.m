function thumbXacroGUI
    % GUI for controlling and retargeting human thumb to robot thumb

    %% Build Models
    humanThumb = buildThumb();
    robotThumb = buildThumbXacro();

    %% Create GUI Figure and Axes
    % Adjust the figure size as desired.
    fig = figure('Name', 'Thumb Control GUI', 'NumberTitle', 'off', ...
                 'Position', [100, 100, 1300, 800]);
    
    % Instead of default subplots, specify custom positions:
    % For example, let the human thumb axes occupy 45% of the width starting at 5%
    % and 80% of the height starting at 15% from the bottom.
    axHuman = subplot('Position', [0.05, 0.35, 0.4, 0.5]);
    title(axHuman, 'Human Thumb');
    grid(axHuman, 'on');
    hold(axHuman, 'on');
    set(axHuman, 'CameraViewAngleMode', 'manual');

    % Let the robot thumb axes occupy the right half (from 55% to 100% of width)
    axRobot = subplot('Position', [0.55, 0.15, 0.3, 0.6]);
    title(axRobot, 'Robot Thumb');
    grid(axRobot, 'on');
    hold(axRobot, 'on');
    set(axRobot, 'CameraViewAngleMode', 'manual');

    %% Create Sliders and Angle Display Texts for Human Thumb
    numSliders = 4;
    % Define different range for each slider as [min, max] (in radians)
    sliderRanges = [ -pi/3, pi/3;
                     -pi/2, pi/2;
                     0.0, pi/2;
                     0.0, pi/2 ];
    
    % Preallocate arrays for slider and text handles.
    slidersHuman = gobjects(1, numSliders);
    sliderTexts  = gobjects(1, numSliders);
    
    % Adjust these normalized positions for the control elements.
    sliderX = 0.1;        % Left edge of slider (in the figure)
    sliderYStart = 0.15;  % Starting y for first slider (closer to bottom)
    sliderWidth = 0.3;
    sliderHeight = 0.03;
    textX = sliderX + sliderWidth + 0.02;  % Place text to the right of slider
    textWidth = 0.1;
    
    for i = 1:numSliders
        sliderY = sliderYStart - (i-1)*0.04;
        % Create slider control with individual ranges.
        slidersHuman(i) = uicontrol('Style', 'slider', ...
            'Min', sliderRanges(i,1), 'Max', sliderRanges(i,2), 'Value', 0, ...
            'Units', 'normalized', ...
            'Position', [sliderX, sliderY, sliderWidth, sliderHeight], ...
            'Callback', @updateThumb);
        % Create text control to display current slider value.
        sliderTexts(i) = uicontrol('Style', 'text', ...
            'Units', 'normalized', ...
            'Position', [textX, sliderY, textWidth, sliderHeight], ...
            'String', '0');
    end
    
    %% Update Function: Called when any slider changes
    function updateThumb(~, ~)
        % Read current human thumb joint angles from the sliders.
        q_human = zeros(1, numSliders);
        for i = 1:numSliders
            q_human(i) = get(slidersHuman(i), 'Value');
            % Update the corresponding text with the current value (formatted to 2 decimals)
            set(sliderTexts(i), 'String', num2str(q_human(i), '%.2f'));
        end

        % Compute human thumb joint positions.
        % Assumed order:
        %   Row 1: Joint 1 (e.g., CMC Bending)
        %   Row 2: Joint 2 (e.g., CMC Side)
        %   Row 3: Joint 3 (e.g., MCP Bending)
        %   Row 4: Joint 4 (e.g., IP Bending)
        %   Row 5: Fingertip
        humanJoints = getHumanThumbPositions(q_human, humanThumb);
        
        % Define a flexible mapping.
        mapping.humanIndices = [2, 3];  % (Indices from the human joint matrix)
        mapping.robotIndices = [1, 3, 4];    % (Indices from the robot joint matrix)
        mapping.targetLink   = 'thumb_fingertip'; % (if needed for IK)
        
        % Call the unified retargeting function.
        q_robot = retargetThumb(humanJoints, robotThumb, mapping);
        
        % --- Directly override robot joints 3 and 4 with a tunable gain ---
        gain = -1.0;  % Adjust this gain as needed.
        q_robot(3) = gain * q_human(3);
        q_robot(4) = gain * q_human(4);

        % Compute robot thumb joint positions.
        robotJoints = getRobotThumbPositions(q_robot, robotThumb);
        
        % Update Human Thumb Plot
        cla(axHuman);
        plotThumb(axHuman, humanJoints);
        humanControlPoints = humanJoints(mapping.humanIndices, :);
        plotSpline(axHuman, humanControlPoints, 'b');
        text(axHuman, 0, 0, 0, 'Origin', 'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');
        if size(humanJoints,1) >= 5
            humanFingertip = humanJoints(5, :);
            text(axHuman, humanFingertip(1), humanFingertip(2), humanFingertip(3), ...
                 'Fingertip', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
        end
        
        % Update Robot Thumb Plot
        cla(axRobot);
        plotThumb(axRobot, robotJoints);
        robotControlPoints = robotJoints(mapping.robotIndices, :);
        plotSpline(axRobot, robotControlPoints, 'r');
        text(axRobot, 0, 0, 0, 'Origin', 'Color', 'm', 'FontSize', 12, 'FontWeight', 'bold');
        if size(robotJoints,1) >= 6
            robotFingertip = robotJoints(6, :);
            text(axRobot, robotFingertip(1), robotFingertip(2), robotFingertip(3), ...
                 'Fingertip', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
        end

        view(axHuman, [-45 30]);
        view(axRobot, [-45 30]);
    end
    
    % Initial update so the GUI displays initial plots.
    updateThumb();
end
