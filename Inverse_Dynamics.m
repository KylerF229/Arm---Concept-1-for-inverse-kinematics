function [M,C,G] = Inverse_Dynamics(q,dq,p)

% coordinates
q1 = q(1);
q2 = q(2);
q3 = q(3);

% velocities
dq1 = dq(1);
dq2 = dq(2);
dq3 = dq(3);

% MASS MATRIX CONSTANTS
A = p.J1+p.L1^2*(p.m1/4+p.mM2+p.m2+p.m3+p.m_gripper+p.m_payload);
B = p.J2+p.L2^2*(p.m2/4+p.m3+p.m_gripper+p.m_payload);
E = p.J3+p.Gripper^2*(p.m_gripper/4+p.m_payload);

D12 = p.L1*p.L2*(p.m2/2+p.m3+p.m_gripper+p.m_payload);
D13 = p.L1*p.Gripper*(p.m_gripper/2+p.m_payload);
D23 = p.L2*p.Gripper*(p.m_gripper/2+p.m_payload);

% MASS MATRIX
M = zeros(3,3);
M(1,1) = A;
M(1,2) = D12*cos(q1-q2);
M(1,3) = D13*cos(q1-q3);
M(2,1) = M(1,2);
M(2,2) = B;
M(2,3) = D23*cos(q2-q3);
M(3,1) = M(1,3);
M(3,2) = M(2,3);
M(3,3) = E;

% CORIOLIS / CENTRIFUGAL MATRIX
C = zeros(3,3);
C(1,2) = D12*sin(q1-q2)*dq2;
C(1,3) = D13*sin(q1-q3)*dq3;
C(2,1) = -D12*sin(q1-q2)*dq1;
C(2,3) = D23*sin(q2-q3)*dq3;
C(3,1) = -D13*sin(q1-q3)*dq1;
C(3,2) = -D23*sin(q2-q3)*dq2;

% GRAVITY VECTOR
G = zeros(3,1);
G(1) = p.g*p.L1*cos(q1)*(p.m1/2+p.mM2+p.m2+p.m3+p.m_gripper+p.m_payload);
G(2) = p.g*p.L2*cos(q2)*(p.m2/2+p.m3+p.m_gripper+p.m_payload);
G(3) = p.g*p.Gripper*cos(q3)*(p.m_gripper/2+p.m_payload);

end