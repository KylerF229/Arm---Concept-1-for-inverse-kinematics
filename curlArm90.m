function [t,q_des,dq_des,ddq_des,Q_required,Q_inertia,Q_velocity,Q_gravity,maxTorque] = curlArm90(p)

% CURL ARM BY 90 DEGREES AT EACH JOINT
% Start with arm fully extended horizontally.
% Move shoulder, elbow, and wrist sequentially.
% Absolute link angles are used for dynamics.

% SETTINGS
T = p.curl.timePerJoint;
Ns = p.curl.samples;
direction = p.curl.direction;

if T <= 0 || Ns < 2 || numel(direction) ~= 3
    error('Invalid curl settings')
end

% RELATIVE JOINT ANGLES TO ABSOLUTE LINK ANGLES
B = tril(ones(3));

% INITIAL CONDITIONS
relative0 = zeros(3,1);
t = [];
q_des = [];
dq_des = [];
ddq_des = [];

% SEQUENTIAL JOINT ROTATIONS
for joint = 1:3

    ts = linspace(0,T,Ns)';

    if joint > 1
        ts = ts(2:end);
    end

    u = ts/T;

    % FIFTH ORDER POLYNOMIAL
    % Zero velocity and acceleration at segment endpoints
    s = 10*u.^3-15*u.^4+6*u.^5;
    ds = (30*u.^2-60*u.^3+30*u.^4)/T;
    dds = (60*u-180*u.^2+120*u.^3)/T^2;

    % RELATIVE JOINT TRAJECTORY
    relative = repmat(relative0',length(ts),1);
    drelative = zeros(length(ts),3);
    ddrelative = zeros(length(ts),3);

    delta = direction(joint)*pi/2;

    relative(:,joint) = relative0(joint)+delta*s;
    drelative(:,joint) = delta*ds;
    ddrelative(:,joint) = delta*dds;

    % CONVERT TO ABSOLUTE ANGLES
    q = relative*B';
    dq = drelative*B';
    ddq = ddrelative*B';

    % STORE TRAJECTORY
    t = [t;(joint-1)*T+ts];
    q_des = [q_des;q];
    dq_des = [dq_des;dq];
    ddq_des = [ddq_des;ddq];

    % FINAL RELATIVE ANGLE FOR NEXT SEGMENT
    relative0(joint) = relative0(joint)+delta;

end

% TORQUE ARRAYS
N = length(t);
Q_inertia = zeros(N,3);
Q_velocity = zeros(N,3);
Q_gravity = zeros(N,3);

% INVERSE DYNAMICS
for k = 1:N

    q = q_des(k,:)';
    dq = dq_des(k,:)';
    ddq = ddq_des(k,:)';

    [M,C,G] = Inverse_Dynamics(q,dq,p);

    % GENERALIZED TORQUES IN ABSOLUTE COORDINATES
    QI = M*ddq;
    QV = C*dq;
    QG = G;

    % CONVERT TO PHYSICAL MOTOR TORQUES
    Q_inertia(k,:) = (B'*QI)';
    Q_velocity(k,:) = (B'*QV)';
    Q_gravity(k,:) = (B'*QG)';

end

% TOTAL REQUIRED MOTOR TORQUE
Q_required = Q_inertia+Q_velocity+Q_gravity;

% MAXIMUM ABSOLUTE TORQUE FOR EACH MOTOR
[maxTorque,index] = max(abs(Q_required),[],1);

% TORQUE RATINGS WITH FACTOR OF SAFETY
ratedTorque = p.FOS*maxTorque;

% OUTPUT RESULTS
fprintf('\n====================================\n');
fprintf('90 DEGREE ARM CURL TORQUE ANALYSIS\n');
fprintf('====================================\n');

fprintf('\nInitial Static Torque (N*m)\n');
for motor = 1:3
    fprintf('Motor %d: %.3f N*m\n',motor,Q_gravity(1,motor));
end

fprintf('\nMaximum Dynamic Torque (N*m)\n');
for motor = 1:3
    fprintf('Motor %d: %.3f N*m at %.2f s\n',motor,maxTorque(motor),t(index(motor)));
end

fprintf('\nRequired Torque With FOS %.1f\n',p.FOS);
for motor = 1:3
    fprintf('Motor %d: %.3f N*m\n',motor,ratedTorque(motor));
end

fprintf('====================================\n');

end
