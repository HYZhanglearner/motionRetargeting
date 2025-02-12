function positions = getRobotThumbPositions(q, robotThumb)
% Computes joint positions for the robot thumb using `buildThumbXacro`

% Extract transforms for each joint
TBase = getTransform(robotThumb, q, 'base_link');
T1 = getTransform(robotThumb, q, 'thumb_metacarpal');
T2 = getTransform(robotThumb, q, 'thumb_actuators');
T3 = getTransform(robotThumb, q, 'thumb_proximal');
T4 = getTransform(robotThumb, q, 'thumb_distal');
Tip = getTransform(robotThumb, q, 'thumb_fingertip');

% Convert transforms to positions
positions = [tform2trvec(TBase); tform2trvec(T1); tform2trvec(T2); ...
             tform2trvec(T3); tform2trvec(T4); tform2trvec(Tip)];

% Flip Z-axis
reflectionMatrix = [1, 0, 0; 0, 1, 0; 0, 0, 1];
positions = (reflectionMatrix * positions')';
end
