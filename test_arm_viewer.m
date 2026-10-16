function test_arm_viewer
% Field mapping for Kyler's actual parameter names; explicit overrides win.
source=struct('Gripper',.21,'m_gripper',.12,'m_payload',3);
mapped=arm_viewer_parameters(source);
assert(mapped.viewer.gripperLength==.21 && mapped.viewer.gripperMass==.12);
assert(mapped.viewer.payloadMass==3);
source.viewer.payloadMass=2;
mapped=arm_viewer_parameters(source); assert(mapped.viewer.payloadMass==2);
source.rover.mountHeight=.6096;
mapped=arm_viewer_parameters(source); assert(mapped.viewer.mountHeight==.6096);
% Actual arm lengths: solve task poses and check FK, including tool angle.
task=arm_viewer_parameters(struct('L1',.3702,'L2',.3702));
task.viewer.mountHeight=.6096;
for row=[.50 0 .03 -90; .55 0 .50 0]'
 [sol,msg]=arm_target_pose(row(1:3),row(4),zeros(3,1),task);
 assert(~isempty(sol),msg); actual=arm_static_pose(sol,task);
 assert(norm(actual.tcpXYZ-row(1:3)')<1e-9);
 assert(abs(mod(sol(3)*180/pi-row(4)+180,360)-180)<1e-8);
end
[bad,~]=arm_target_pose([.5 .1 .03],-90,zeros(3,1),task); assert(isempty(bad));
[bad,~]=arm_target_pose([5 0 .03],-90,zeros(3,1),task); assert(isempty(bad));
[high,~]=arm_target_pose([.55 0 .50],0,zeros(3,1),task,2);
[low,~]=arm_target_pose([.55 0 .50],0,zeros(3,1),task,3);
assert(sin(high(1))>sin(low(1)),'Elbow branch selection failed');
% Compare with the current project dynamics, if available on the path.
if exist('Inverse_Dynamics','file')==2
 check=task; check.Gripper=check.viewer.gripperLength;
 check.m_gripper=.1; check.m_payload=5;
 check.viewer.gripperMass=check.m_gripper; check.viewer.payloadMass=check.m_payload;
 check.J1=.01; check.J2=.01; check.J3=.05;
 [~,~,G]=Inverse_Dynamics(high,zeros(3,1),check);
 actual=arm_static_pose(high,check);
 assert(norm(actual.Gabs-G)<1e-9,'Current dynamics and viewer gravity differ');
end
p=arm_viewer_parameters(struct());
q=[.3;-.5;.8]; s=arm_static_pose(q,p); h=1e-6; numeric=zeros(3,1);
for k=1:3
 e=zeros(3,1); e(k)=h;
 a=arm_static_pose(q+e,p); b=arm_static_pose(q-e,p);
 numeric(k)=(a.potential-b.potential)/(2*h);
end
assert(norm(numeric-s.Gabs)<1e-6,'Potential-energy derivative mismatch');
assert(norm(s.tauJoint-[sum(s.Gabs);sum(s.Gabs(2:3));s.Gabs(3)])<1e-10);
% Legacy dynamics gravity reproduced exactly with no offset masses.
p.viewer.payloadMass=0; p.viewer.gripperMass=0;
s=arm_static_pose(q,p);
legacy=p.g*[p.L1*cos(q(1))*(p.m1/2+p.mM2+p.m2+p.m3); ...
 p.L2*cos(q(2))*(p.m2/2+p.m3);0];
assert(norm(s.Gabs-legacy)<1e-10);
% Isolated TCP payload: torque equals weight * horizontal lever arm.
p.m1=0;p.m2=0;p.mM2=0;p.m3=0;
s=arm_static_pose(zeros(3,1),p,2);
assert(norm(s.tauJoint-2*p.g*[p.L1+p.L2+.15;p.L2+.15;.15])<1e-10);
% Exercise graphics and absolute-angle trajectory adapter.
p=arm_viewer_parameters(struct());
v=plotArm3D([0 0 0],p); v.setPose(q); assert(norm(v.getPose()-q)<1e-10);
v.setTarget([.5 0 .03],-90); assert(v.solveTarget());
s=v.getResults(); assert(norm(s.tcpXYZ-[.5 0 .03])<1e-9);
v.recordPose(); saved=v.getRecords(); assert(height(saved)==1);
assert(norm(saved{1,{'J1_Nm','J2_Nm','J3_Nm'}}-s.tauJoint')<1e-10);
qgood=v.getPose(); v.setTarget([5 0 .03],-90); assert(~v.solveTarget());
assert(norm(v.getPose()-qgood)<1e-10);
v.setTarget([.5 0 .03],-90);
exportapp(v.figure,fullfile(fileparts(mfilename('fullpath')),'viewer_preview.png'));
close(v.figure); p.plot.model3D=false; assert(isempty(arm_pose_viewer(p)));
disp('All static torque and viewer smoke checks passed.');
end
