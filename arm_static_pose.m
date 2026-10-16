function s = arm_static_pose(q,p,payloadMass)
% q: three ABSOLUTE angles in radians, CCW from +X toward +Z.
% Gabs = dU/dq; tauJoint = T' * Gabs, q=T*relativeAngles.
% Positive holding torque acts CCW in the X/Z plane (axis -world Y).
if nargin<3, payloadMass=p.viewer.payloadMass; end
q=q(:); assert(numel(q)==3 && all(isfinite(q)),'q must contain 3 finite angles.');
v=p.viewer; L=[p.L1 p.L2 v.gripperLength];
assert(all(L>0) && all(isfinite(L)),'Lengths must be positive.');
assert(all([p.m1 p.m2 p.mM2 p.m3 v.gripperMass payloadMass]>=0),'Masses must be nonnegative.');
u=[cos(q) sin(q)]; du=[-sin(q) cos(q)];
P=zeros(4,2); P(1,:)=v.baseXZ;
for k=1:3, P(k+1,:)=P(k,:)+L(k)*u(k,:); end
% Each physical mass appears exactly once. mM2 at elbow; m3 at wrist.
mass=[p.m1 p.mM2 p.m2 p.m3 v.gripperMass payloadMass];
com=[P(1,:)+L(1)/2*u(1,:); P(2,:); ...
    P(2,:)+L(2)/2*u(2,:); P(3,:); ...
    P(3,:)+v.gripperCOM*u(3,:); ...
    P(4,:)+v.payloadOffset(1)*u(3,:)+v.payloadOffset(2)*du(3,:)];
% Vertical derivatives of each COM with respect to absolute q1,q2,q3.
D=[L(1)/2*cos(q(1)) 0 0; L(1)*cos(q(1)) 0 0; ...
 L(1)*cos(q(1)) L(2)/2*cos(q(2)) 0; ...
 L(1)*cos(q(1)) L(2)*cos(q(2)) 0; ...
 L(1)*cos(q(1)) L(2)*cos(q(2)) v.gripperCOM*cos(q(3)); ...
 L(1)*cos(q(1)) L(2)*cos(q(2)) ...
 (L(3)+v.payloadOffset(1))*cos(q(3))-v.payloadOffset(2)*sin(q(3))];
s.Gabs=p.g*D'*mass';
s.tauJoint=tril(ones(3))'*s.Gabs;
s.potential=p.g*mass*com(:,2);
s.pointsXYZ=[P(:,1) zeros(4,1) P(:,2)+v.mountHeight];
s.tcpXYZ=s.pointsXYZ(4,:); s.comXZ=com;
end
