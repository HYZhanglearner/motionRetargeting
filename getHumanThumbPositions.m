function positions = getHumanThumbPositions(q, humanThumb)
% Computes joint positions for the human thumb using `buildThumb`

% Extract transforms for each joint
T1_bend = getTransform(humanThumb, q, 'body1');  % CMC Bending
T1_side = getTransform(humanThumb, q, 'body2');  % CMC Side
T2 = getTransform(humanThumb, q, 'body4');       % MCP
T3 = getTransform(humanThumb, q, 'body6');       % IP
Tip = getTransform(humanThumb, q, 'link3');      % Fingertip

% Convert transforms to positions
positions = [tform2trvec(T1_bend); tform2trvec(T1_side); tform2trvec(T2); ...
             tform2trvec(T3); tform2trvec(Tip)];
end
