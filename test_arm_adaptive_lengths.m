function test_arm_adaptive_lengths
p=arm_viewer_parameters(struct('L1',.3702,'L2',.3702));
fields={'TargetX_m','TargetY_m','TargetZ_m','TargetAngle_deg','Payload_kg'};
far=array2table([1.45 0 .60 0 5],'VariableNames',fields);
o=struct('L1Values',.2:.1:.6,'L2Values',.2:.1:.6, ...
 'checkClearance',false,'maxLinkLength',1);
fixed=arm_optimize_lengths(p,far,o); assert(isempty(fixed.bestP));
r=arm_optimize_recorded_targets(p,far,o);
assert(~isempty(r.bestP) && r.expansion.attempts>1);
s=arm_static_pose(r.poses(1,:)',r.bestP); assert(norm(s.tcpXYZ-far{1,1:3})<1e-9);
o.autoExpand=false; fixed=arm_optimize_recorded_targets(p,far,o); assert(isempty(fixed.bestP));
o.autoExpand=true; o.maxLinkLength=.8;
impossible=far; impossible.TargetX_m=5;
none=arm_optimize_recorded_targets(p,impossible,o);
assert(isempty(none.bestP) && max(none.expansion.searchedMax)<=.8+1e-9);
% An already-open study must see a new target; Apply must not use stale results.
v=plotArm3D([0 0 0],p); clean=onCleanup(@()closeIfOpen(v.figure));
v.setTarget([.5 0 .6],0); v.recordPose();
study=arm_length_study(p,v.getRecords,v.applyLengthStudy);
cleanStudy=onCleanup(@()closeIfOpen(study.figure));
study.run(); first=study.getResult(); assert(height(first.targets)==1);
v.setTarget([1.45 0 .6],0); assert(~v.solveTarget()); v.recordPose();
study.apply(); latest=study.getResult();
assert(height(latest.targets)==2 && ~isempty(latest.bestP));
assert(latest.expansion.attempts>1);
s=v.getResults(); assert(norm(s.tcpXYZ-[1.45 0 .6])<1e-9);
assert(latest.bestP.L1+latest.bestP.L2>first.bestP.L1+first.bestP.L2);
fprintf('PASS: newly recorded unreachable target included; search expanded; viewer reached new target.\n');
end
function closeIfOpen(f)
if isgraphics(f), close(f); end
end
