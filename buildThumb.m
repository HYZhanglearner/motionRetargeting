function thumb = buildThumb(L1, L2, L3)
    if nargin < 3
        L1 = 78.15; L2 = 53.16; L3 = 49.43;
    end
% BUILDTHUMB Creates a rigidBodyTree model for a human thumb.
%
%   thumb = buildThumb(L1, L2, L3) returns a rigidBodyTree with four
%   DOFs modeling the thumb. The segments (links) have lengths L1, L2, and L3.
%
%   The DOFs are:
%     1. CMC Bending (revolute about z)
%     2. CMC Side    (revolute about y)
%     3. MCP Bending (revolute about z)
%     4. IP  Bending (revolute about z)
%
%   The thumb’s base (CMC) is located at the origin.

% Create a rigid body tree with row-vector configuration
thumb = robotics.RigidBodyTree('DataFormat','row','MaxNumBodies',7);

%% Base and CMC Joint (2 DOFs)
% Joint 1: CMC Bending
body1 = robotics.RigidBody('body1');
joint1 = robotics.Joint('joint1','revolute');
joint1.HomePosition = 0;
joint1.JointAxis = [0 0 -1];  % Rotation about z-axis
setFixedTransform(joint1,trvec2tform([0 0 0])); % At the origin
body1.Joint = joint1;
addBody(thumb, body1, 'base');

% Joint 2: CMC Side (added in series at the same physical location)
body2 = robotics.RigidBody('body2');
joint2 = robotics.Joint('joint2','revolute');
joint2.HomePosition = 0;
joint2.JointAxis = [0 1 0]; % Rotation about y-axis
setFixedTransform(joint2,trvec2tform([0 0 0])); % No translation
body2.Joint = joint2;
addBody(thumb, body2, 'body1');

%% Link 1: From CMC to MCP
body3 = robotics.RigidBody('link1');
joint_fixed1 = robotics.Joint('fix1','fixed');
% Translate by L1 along the x-axis
setFixedTransform(joint_fixed1,trvec2tform([L1 0 0]));
body3.Joint = joint_fixed1;
addBody(thumb, body3, 'body2');

%% MCP Joint (1 DOF)
body4 = robotics.RigidBody('body4');
joint3 = robotics.Joint('joint3','revolute');
joint3.HomePosition = 0;
joint3.JointAxis = [0 0 -1]; % Rotation about z-axis
setFixedTransform(joint3,trvec2tform([0 0 0])); % At the end of link1
body4.Joint = joint3;
addBody(thumb, body4, 'link1');

%% Link 2: From MCP to IP
body5 = robotics.RigidBody('link2');
joint_fixed2 = robotics.Joint('fix2','fixed');
% Translate by L2 along the x-axis
setFixedTransform(joint_fixed2,trvec2tform([L2 0 0]));
body5.Joint = joint_fixed2;
addBody(thumb, body5, 'body4');

%% IP Joint (1 DOF)
body6 = robotics.RigidBody('body6');
joint4 = robotics.Joint('joint4','revolute');
joint4.HomePosition = 0;
joint4.JointAxis = [0 0 -1]; % Rotation about z-axis
setFixedTransform(joint4,trvec2tform([0 0 0])); % At the end of link2
body6.Joint = joint4;
addBody(thumb, body6, 'link2');

%% Link 3: Thumb Tip
body7 = robotics.RigidBody('link3');
joint_fixed3 = robotics.Joint('fix3','fixed');
% Translate by L3 along the x-axis
setFixedTransform(joint_fixed3,trvec2tform([L3 0 0]));
body7.Joint = joint_fixed3;
addBody(thumb, body7, 'body6');

end
