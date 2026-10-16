function p = arm_parameters(p)
% ARM_PARAMETERS
% 3D model parameters for the fixed-base robotic arm.
% Two linkages, three rotational joints, and a gripper.

% CHECK EXISTING PARAMETERS
required = {'L1','L2','Gripper','g','m0','m1','mM2',...
    'm2','m3','m_gripper','m_payload','Dtube','t'};

for k = 1:numel(required)
    assert(isfield(p,required{k}),...
        'Missing parameter: p.%s',required{k});
end

% ROVER DIMENSIONS
p.rover.mountHeight = 0.6096; % m
p.rover.mountBehindFront = 0.35; % m
p.rover.mountLeftOfCenter = 0; % m
p.rover.width = 0.80; % m
p.rover.length = 1.00; % m
p.rover.deckHeight = 0.50; % m
p.rover.groundClearance = 0.15; % m

% ARM GEOMETRY
p.geometry.baseRise = 0; % Base at rover mount
p.geometry.upperArm = p.L1;
p.geometry.forearm = p.L2;
p.geometry.wristExtension = 0;
p.geometry.gripperLength = p.Gripper;
p.geometry.linkRadius = p.Dtube/2;
p.geometry.wallThickness = p.t;

% GRIPPER GEOMETRY
% Wrist joint is located at end of Link 2.
% The gripper extends from the wrist.
p.tool.wristToGrasp = eye(4);
p.tool.wristToGrasp(1,4) = p.Gripper;
p.tool.mass = p.m_gripper;
p.tool.comInWrist = [p.Gripper/2 0 0];
p.tool.radius = p.geometry.linkRadius;

% PAYLOAD
p.payload.masses = [0 1 3 5];
p.payload.viewerMass = p.m_payload;
p.payload.comInTool = [0 0 0];

% GRAVITY
p.gravity = [0 0 -p.g];

% CAMERA
p.camera.atJoint = 3;
p.camera.mass = p.m_camera;

% JOINT MODEL
% Positive rotation about local -Y lifts the link toward +Z.
% The model uses RELATIVE angles between consecutive links.
% Your existing inverse dynamics uses ABSOLUTE angles.

joint = struct('name','','axis',[0 -1 0],...
    'fixedTransform',eye(4),...
    'limitsDeg',[-180 180],...
    'homeDeg',0,...
    'preferredDeg',0,...
    'actuatorMass',0,...
    'linkVector',[0 0 0],...
    'mass',0,...
    'com',[0 0 0],...
    'radius',p.geometry.linkRadius);

j = repmat(joint,1,3);

% MOTOR 1 / SHOULDER / LINK 1
j(1).name = 'Motor 1 - Shoulder';
j(1).axis = [0 -1 0];
j(1).actuatorMass = p.m0;
j(1).linkVector = [p.L1 0 0];
j(1).mass = p.m1;
j(1).com = [p.L1/2 0 0];

% MOTOR 2 / ELBOW / LINK 2
j(2).name = 'Motor 2 - Elbow';
j(2).axis = [0 -1 0];
j(2).actuatorMass = p.mM2;
j(2).linkVector = [p.L2 0 0];
j(2).mass = p.m2;
j(2).com = [p.L2/2 0 0];

% MOTOR 3 / WRIST
j(3).name = 'Motor 3 - Wrist';
j(3).axis = [0 -1 0];
j(3).actuatorMass = p.m3;
j(3).linkVector = [0 0 0];
j(3).mass = 0;
j(3).com = [0 0 0];

% BUILD 3D MODEL
p.models = struct(...
    'name','Fixed Base 3 Joint Arm',...
    'joints',j,...
    'torqueLimitsNm',nan(1,3));

% UNKNOWN TORQUE LIMITS REMAIN NaN
% Joint limits are placeholders until hardware is selected.

end
