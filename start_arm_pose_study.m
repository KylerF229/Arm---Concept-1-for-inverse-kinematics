% Pose study using the physical properties in your current ARM_1.m.
% Keep this file beside your existing arm_parameters.m and viewer files.
p = struct(); p.plotOutputs = 1; p.plot.model3D = 1;
% PROPERTIES
p.g = 9.81; %m/s^2
p.density = 2700; 

% Link lengths
p.L1 = 0.3702;  % m
p.L2 = 0.3702;  % m
p.Gripper = 0.15; % m
p.Ltotal = p.L1+p.L2+p.Gripper;

% Link Dimensions
p.Dtube = 0.0254; % m - Outer diameter
p.t = 0.003; % m - Tube wall thickness
p.Ro = p.Dtube/2;
p.Ri = p.Ro-p.t;
p.A = pi*(p.Ro^2-p.Ri^2); % m^2

% Link Mass
p.m1_tube = p.density*p.A*p.L1;
p.m2_tube = p.density*p.A*p.L2;
p.m_gripper_tube = p.density*p.A*p.Gripper;

% Mass
p.m0 = 1.4; % kg - Base motor
p.mM2 = 1.4; % kg - Motor 2
p.mM3 = 2.8; % kg - Motors 3 and 4 combined
p.m_gripper_actuator = 1; % kg
p.m_camera = 1; % kg
p.m_encoders = 0; % kg - Total encoder mass
p.m_electronics = 0.1; % kg - Wiring/electronics
p.m_payload = 5; % kg

% Wiring distributed
% Assumed uniformly distributed along the entire arm
p.m_wire1 = p.m_electronics*p.L1/p.Ltotal;
p.m_wire2 = p.m_electronics*p.L2/p.Ltotal;
p.m_wire_gripper = p.m_electronics*p.Gripper/p.Ltotal;

% Total Linkage Masses
p.m1 = p.m1_tube+p.m_wire1;
p.m2 = p.m2_tube+p.m_wire2;
p.m_gripper = p.m_gripper_tube+p.m_wire_gripper;

% MASS LOCATED AT END OF LINK 2
p.m3 = p.mM3+p.m_gripper_actuator+p.m_camera;

% MOMENT OF INERTIA
% Uniform slender rod 
% J = (1/12)*m*L^2
% These are about the center of mass of each link.
p.J1 = (1/12)*p.m1*p.L1^2;
p.J2 = (1/12)*p.m2*p.L2^2;
% End-effector rotational inertia
% (TEmp) change this when we know actual geometry from CAD
p.J3 = 0.05; % kg*m^2
p = arm_parameters(p);


viewer = plotArm3D([0 0 0],p);
