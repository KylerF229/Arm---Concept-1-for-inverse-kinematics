function test_arm_length_study
% Numerical and UI regression checks; no Robotics Toolbox required.
p=struct('L1',.3702,'L2',.3702,'Gripper',.15,'g',9.81,'density',2700, ...
 'Dtube',.0254,'t',.003,'m_electronics',.1,'mM2',1.4,'m3',4.8,'m_payload',5);
p.A=pi*((p.Dtube/2)^2-(p.Dtube/2-p.t)^2);
p.Ltotal=p.L1+p.L2+p.Gripper;
p.m1=p.density*p.A*p.L1+p.m_electronics*p.L1/p.Ltotal;
p.m2=p.density*p.A*p.L2+p.m_electronics*p.L2/p.Ltotal;
p.m_gripper=p.density*p.A*p.Gripper+p.m_electronics*p.Gripper/p.Ltotal;
p.rover.mountHeight=.6096; p=arm_viewer_parameters(p);
new=arm_resize_links(p,.25,.50);
assert(abs(new.m1-(p.density*p.A*.25+.1*.25/.90))<1e-12);
assert(abs(new.m_wire1+new.m_wire2+new.m_wire_gripper-.1)<1e-12);
assert(abs(new.J1-new.m1*.25^2/12)<1e-12);
assert(new.m3==p.m3 && new.mM2==p.mM2 && new.Gripper==.15);
targets=array2table([.50 0 .03 -90 5;.55 0 .50 0 5], ...
 'VariableNames',{'TargetX_m','TargetY_m','TargetZ_m','TargetAngle_deg','Payload_kg'});
o=struct('L1Values',.20:.025:.60,'L2Values',.20:.025:.60);
r=arm_optimize_lengths(p,targets,o);
assert(~isempty(r.bestP),r.message);
assert(abs(r.bestP.L1+r.bestP.L2-min(r.ranked.Total_m))<1e-10);
assert(all(r.ranked.TargetsReached==2));
if r.baseline.targetsReached==height(targets)
 assert(r.bestP.L1+r.bestP.L2<=p.L1+p.L2+1e-9,'Must not overlook a shorter feasible current arm');
end
for k=1:2
 s=arm_static_pose(r.poses(k,:)',r.bestP,targets.Payload_kg(k));
 assert(norm(s.tcpXYZ-targets{k,1:3})<1e-9);
 assert(norm(s.tauJoint'-r.targetResults{k,{'J1_Nm','J2_Nm','J3_Nm'}})<1e-9);
end
% Independent analytical reach test with clearance disabled and wide limits.
check=o; check.checkClearance=false;
wide=p; wide.viewer.limitsDeg=repmat([-720 720],3,1);
allReach=arm_optimize_lengths(wide,targets,check);
for k=1:height(allReach.candidates)
 a=allReach.candidates.L1_m(k); b=allReach.candidates.L2_m(k);
 wristX=targets.TargetX_m-p.Gripper*cosd(targets.TargetAngle_deg);
 wristZ=targets.TargetZ_m-p.viewer.mountHeight-p.Gripper*sind(targets.TargetAngle_deg);
 d=hypot(wristX,wristZ);
 expected=all(d<=a+b+1e-9 & d>=abs(a-b)-1e-9);
 assert(allReach.candidates.Feasible(k)==expected);
end
bad=targets; bad.TargetX_m(:)=5;
no=arm_optimize_lengths(p,bad,struct('L1Values',.3,'L2Values',.3));
assert(isempty(no.bestP) && isempty(no.ranked));
% A target embedded in the rover must be rejected when clearance is enabled.
inside=targets(1,:); inside.TargetX_m=0; inside.TargetZ_m=.3; inside.TargetAngle_deg=0;
blocked=arm_optimize_lengths(p,inside,struct('L1Values',.5,'L2Values',.5));
assert(isempty(blocked.bestP));
% Viewer recording, search and geometry application.
v=plotArm3D([0 0 0],p); cleanup=onCleanup(@()closeIfOpen(v.figure));
for k=1:2, v.setTarget(targets{k,1:3},targets.TargetAngle_deg(k)); v.recordPose(); end
v.applyLengthStudy(r); displayed=v.getParameters();
assert(displayed.L1==r.bestP.L1 && displayed.L2==r.bestP.L2);
s=v.getResults(); assert(norm(s.tcpXYZ-targets{end,1:3})<1e-9);
v.recordPose(); history=v.getRecords();
assert(history.L1_m(1)==p.L1 && history.L1_m(3)==r.bestP.L1);
study=arm_length_study(p,targets,v.applyLengthStudy);
cleanupStudy=onCleanup(@()closeIfOpen(study.figure));
study.run(); rr=study.getResult(); assert(~isempty(rr.bestP)); study.apply();
exportapp(study.figure,fullfile(fileparts(mfilename('fullpath')),'length_study_preview.png'));
exportapp(v.figure,fullfile(fileparts(mfilename('fullpath')),'optimized_arm_preview.png'));
fprintf('PASS: shortest grid design L1=%.3f, L2=%.3f, total=%.3f m; %d feasible designs.\n', ...
 r.bestP.L1,r.bestP.L2,r.bestP.L1+r.bestP.L2,height(r.ranked));
end
function closeIfOpen(f)
if isgraphics(f), close(f); end
end
