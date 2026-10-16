function result = arm_optimize_lengths(p,targets,options)
% Finite grid search; best SAMPLED design, not a continuous/global optimum.
% Every target includes ground-referenced XYZ, absolute tool angle and payload.
% Default: shortest L1+L2, then lowest worst absolute joint holding torque.
if nargin<3, options=struct(); end
p=arm_viewer_parameters(p);
options=default(options,'L1Values',.20:.01:.60);
options=default(options,'L2Values',.20:.01:.60);
options=default(options,'objective','shortest');
options=default(options,'checkClearance',true);
options=default(options,'includeCurrentLengths',true);
radius=.0127; if isfield(p,'Dtube'), radius=p.Dtube/2; end
options=default(options,'clearanceRadius',radius);
required={'TargetX_m','TargetY_m','TargetZ_m','TargetAngle_deg','Payload_kg'};
assert(istable(targets) && height(targets)>0,'Record at least one target first.');
assert(all(ismember(required,targets.Properties.VariableNames)),'Target table is missing required columns.');
data=targets{:,required};
assert(isnumeric(data) && all(isfinite(data(:))),'Targets and payloads must be finite numeric values.');
assert(all(data(:,5)>=0),'Payload masses must be nonnegative.');
assert(all(abs(data(:,2))<1e-9),'All targets must have Y=0 for this planar arm.');
validateattributes(options.L1Values,{'numeric'},{'real','finite','positive','vector','nonempty'});
validateattributes(options.L2Values,{'numeric'},{'real','finite','positive','vector','nonempty'});
validateattributes(options.clearanceRadius,{'numeric'},{'real','finite','nonnegative','scalar'});
assert(ismember(string(options.objective),["shortest" "torque"]),'Unknown objective.');
a=unique(options.L1Values(:)); b=unique(options.L2Values(:));
% Avoid recommending a longer grid design merely because the existing
% dimensions fall between grid points. Include them when within the bounds.
if options.includeCurrentLengths
    if p.L1>=min(a) && p.L1<=max(a), a=unique([a;p.L1]); end
    if p.L2>=min(b) && p.L2<=max(b), b=unique([b;p.L2]); end
end
assert(numel(a)*numel(b)<=10000,'Use at most 10,000 length combinations; increase the step.');
rows=zeros(numel(a)*numel(b),7); row=0;
for i=1:numel(a)
    for j=1:numel(b)
        candidate=arm_resize_links(p,a(i),b(j));
        [count,peak]=evaluate(candidate,data,options);
        row=row+1; rows(row,:)=[a(i) b(j) a(i)+b(j) count peak];
    end
end
candidates=array2table(rows,'VariableNames',{'L1_m','L2_m','Total_m','TargetsReached','PeakJ1_Nm','PeakJ2_Nm','PeakJ3_Nm'});
candidates.Feasible=candidates.TargetsReached==height(targets);
candidates.WorstJoint_Nm=max(candidates{:,5:7},[],2);
% Round length score only to make equal sums tie consistently on decimal grids.
candidates.LengthScore=round(candidates.Total_m*1e10)/1e10;
feasible=candidates(candidates.Feasible,:);
if strcmp(options.objective,'shortest'), sortFields={'LengthScore','WorstJoint_Nm','L1_m'};
else, sortFields={'WorstJoint_Nm','LengthScore','L1_m'}; end
feasible=sortrows(feasible,sortFields); feasible.LengthScore=[]; candidates.LengthScore=[];
[baseCount,basePeak,baseQ,baseTorque]=evaluate(p,data,options);
result=struct('options',options,'targets',targets,'candidates',candidates, ...
    'ranked',feasible,'bestP',[],'poses',[],'targetResults',table(), ...
    'baseline',struct('L1',p.L1,'L2',p.L2,'targetsReached',baseCount, ...
    'peakTorque',basePeak,'poses',baseQ,'torques',baseTorque));
if isempty(feasible)
    result.message='No candidate reaches all targets with the current angle, joint limits and clearance settings.';
    return
end
result.bestP=arm_resize_links(p,feasible.L1_m(1),feasible.L2_m(1));
[~,~,result.poses,tau]=evaluate(result.bestP,data,options);
result.targetResults=targets(:,required);
if ismember('PoseName',targets.Properties.VariableNames), result.targetResults.PoseName=targets.PoseName; end
result.targetResults.q1Abs_deg=result.poses(:,1)*180/pi;
result.targetResults.q2Abs_deg=result.poses(:,2)*180/pi;
result.targetResults.q3Abs_deg=result.poses(:,3)*180/pi;
result.targetResults.J1_Nm=tau(:,1); result.targetResults.J2_Nm=tau(:,2); result.targetResults.J3_Nm=tau(:,3);
result.message=sprintf('Best sampled design: L1 %.3f m, L2 %.3f m. All %d targets reached.', ...
    result.bestP.L1,result.bestP.L2,height(targets));
end
function s=default(s,k,value)
if ~isfield(s,k), s.(k)=value; end
end
function [count,peak,Q,T]=evaluate(p,data,options)
n=size(data,1); Q=nan(n,3); T=nan(n,3); count=0;
for t=1:n
    best=inf;
    [~,~,solutions]=arm_target_pose(data(t,1:3),data(t,4),zeros(3,1),p);
    for branch=1:size(solutions,2)
        q=solutions(:,branch);
        s=arm_static_pose(q,p,data(t,5));
        if options.checkClearance && ~clearPose(s.pointsXYZ,p.viewer,options.clearanceRadius), continue; end
        score=max(abs(s.tauJoint));
        if score<best, best=score; Q(t,:)=q'; T(t,:)=s.tauJoint'; end
    end
    if isfinite(best), count=count+1; end
end
if count==n, peak=max(abs(T),[],1); else, peak=[inf inf inf]; end
end
function ok=clearPose(P,v,r)
% Link centerlines must clear the ground and an expanded rover body box.
% No self-collision, payload envelope, drawer walls, or path checking.
ok=false; if any(P(:,3)<r-1e-9), return; end
lo=[v.mountBehindFront-v.roverLength -v.roverWidth/2-v.mountLeftOfCenter v.groundClearance]-r;
hi=[v.mountBehindFront v.roverWidth/2-v.mountLeftOfCenter v.deckHeight]+r;
for k=1:3
    a=P(k,:); d=P(k+1,:)-a; enter=0; leave=1; intersects=true;
    for axis=1:3
        if abs(d(axis))<1e-12
            if a(axis)<lo(axis) || a(axis)>hi(axis), intersects=false; break; end
        else
            cuts=sort(([lo(axis) hi(axis)]-a(axis))/d(axis));
            enter=max(enter,cuts(1)); leave=min(leave,cuts(2));
            if enter>leave, intersects=false; break; end
        end
    end
    if intersects, return; end
end
ok=true;
end
