function thumb = buildThumbXacro()
% BUILTHUMBXACRO constructs a rigidBodyTree representing a robot thumb
% based on the provided xacro data. The complete kinematic chain is:
%
%   base_link → [joint1] → thumb_metacarpal →
%   [joint2] → thumb_actuators →
%   [joint3] → thumb_proximal →
%   [joint4] → thumb_distal →
%   [thumb_fingertip_frame (fixed)] → thumb_fingertip
%
% Joint parameters are taken from your xacro snippet.
% Note: The original xacro defined the fingertip joint with parent "index_distal".
% Here, we assume the correct parent is "thumb_distal".

% Create a rigidBodyTree with row-vector configuration.
thumb = robotics.RigidBodyTree('DataFormat','row','MaxNumBodies',7);

%% 1. Add the base_link.
% We attach base_link to the tree’s base with a fixed joint.
bodyBase = robotics.RigidBody('base_link');
jointBase = robotics.Joint('fixed_base','fixed');
setFixedTransform(jointBase, eye(4));  % no offset
bodyBase.Joint = jointBase;
addBody(thumb, bodyBase, thumb.BaseName);

%% 2. Add Joint1: base_link → thumb_metacarpal
bodyMetacarpal = robotics.RigidBody('thumb_metacarpal');
joint1 = robotics.Joint('joint1','revolute');
joint1.JointAxis = [0 0 1];
% From xacro: origin xyz="-0.0115 0.0390 -0.01585", rpy="( -1.3045e-15, 1.5708, 0)"
tform1 = trvec2tform([-0.0115, 0.0390, -0.01585]) * eul2tform([ -1.3045e-15, 1.5708, 0 ], 'XYZ');
setFixedTransform(joint1, tform1);
joint1.PositionLimits = [-0.3490658503988659, 1.2217304763960306];
bodyMetacarpal.Joint = joint1;
addBody(thumb, bodyMetacarpal, 'base_link');

%% 3. Add Joint2: thumb_metacarpal → thumb_actuators
bodyActuators = robotics.RigidBody('thumb_actuators');
joint2 = robotics.Joint('joint2','revolute');
joint2.JointAxis = [0 0 1];
% From xacro: origin xyz="0.05825 0 0.03225", rpy="(-pi, 1.5708, 0)"
tform2 = trvec2tform([-0.05825, 0, 0.03225]) * eul2tform([-pi, 1.5708, 0], 'XYZ');
setFixedTransform(joint2, tform2);
joint2.PositionLimits = [-0.7853981633974483, 0.7853981633974483];
bodyActuators.Joint = joint2;
addBody(thumb, bodyActuators, 'thumb_metacarpal');

%% 4. Add Joint3: thumb_actuators → thumb_proximal
bodyProximal = robotics.RigidBody('thumb_proximal');
joint3 = robotics.Joint('joint3','revolute');
joint3.JointAxis = [0 0 1];
% From xacro: origin xyz="-0.02499792 -0.012 -0.024", rpy="(-pi/2, 0, -pi)"
tform3 = trvec2tform([0.02499792, -0.012, -0.024]) * eul2tform([-pi/2, 0, -pi], 'XYZ');
setFixedTransform(joint3, tform3);
joint3.PositionLimits = [-1.0471975511965976, 1.0471975511965976];
bodyProximal.Joint = joint3;
addBody(thumb, bodyProximal, 'thumb_actuators');

%% 5. Add Joint4: thumb_proximal → thumb_distal
bodyDistal = robotics.RigidBody('thumb_distal');
joint4 = robotics.Joint('joint4','revolute');
joint4.JointAxis = [0 0 1];
% From xacro: origin xyz="0.06 0 0", rpy="(0, 0, -pi)"
tform4 = trvec2tform([-0.06, 0, 0]) * eul2tform([0, 0, -pi], 'XYZ');
setFixedTransform(joint4, tform4);
joint4.PositionLimits = [-2.0943951023931953, 0];
bodyDistal.Joint = joint4;
addBody(thumb, bodyDistal, 'thumb_proximal');

%% 6. Add the Thumb Fingertip:
% The fingertip is attached with a fixed joint.
bodyFingertip = robotics.RigidBody('thumb_fingertip');
jointFingertip = robotics.Joint('thumb_fingertip_frame','fixed');
% From xacro: origin xyz="-0.0761 0.00520213 -0.0025", rpy="(1.5708, 0, -pi)"
% We assume the parent should be "thumb_distal" rather than "index_distal".
tformFingertip = trvec2tform([0.0761, 0.00520213, -0.0025]) * eul2tform([1.5708, 0, -pi], 'XYZ');
setFixedTransform(jointFingertip, tformFingertip);
bodyFingertip.Joint = jointFingertip;
addBody(thumb, bodyFingertip, 'thumb_distal');

end
