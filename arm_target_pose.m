function [q,message,solutions] = arm_target_pose(xyz,angleDeg,currentQ,p,branch)
% Ground-referenced target; planar analytic IK with relative joint limits.
p=arm_viewer_parameters(p); v=p.viewer; q=[]; solutions=zeros(3,0); xyz=xyz(:)';
if nargin<5, branch=1; end % 1 closest, 2 higher elbow, 3 lower elbow
if numel(xyz)~=3 || ~isscalar(angleDeg) || any(~isfinite([xyz angleDeg]))
 message='Enter finite XYZ coordinates and tool angle.'; return
end
if abs(xyz(2))>1e-9
 message='Outside the motion plane: this arm requires target Y = 0.'; return
end
a=angleDeg*pi/180;
w=[xyz(1) xyz(3)-v.mountHeight]-v.baseXZ-v.gripperLength*[cos(a) sin(a)];
c=(sum(w.^2)-p.L1^2-p.L2^2)/(2*p.L1*p.L2);
if abs(c)>1+1e-10
 message='Unreachable at this tool angle. Move the target or change the angle.'; return
end
old=[currentQ(1);diff(currentQ(:))]*180/pi; best=inf;
for beta=[acos(max(-1,min(1,c))) -acos(max(-1,min(1,c)))]
 a1=atan2(w(2),w(1))-atan2(p.L2*sin(beta),p.L1+p.L2*cos(beta));
 r=[a1;beta;a-a1-beta]*180/pi; valid=true;
 for j=1:3
  lo=ceil((v.limitsDeg(j,1)-r(j)-1e-10)/360);
  hi=floor((v.limitsDeg(j,2)-r(j)+1e-10)/360);
  if lo>hi, valid=false; break; end
  r(j)=r(j)+360*min(hi,max(lo,round((old(j)-r(j))/360)));
 end
 score=norm(r-old);
 if branch==2, score=-sin(r(1)*pi/180); end
 if branch==3, score=sin(r(1)*pi/180); end
 if valid
  solutions(:,end+1)=cumsum(r)*pi/180;
  if score<best, q=cumsum(r)*pi/180; best=score; end
 end
end
if isempty(q), message='Geometric solutions exceed configured relative joint limits.';
else, message='Target solved. Holding torques updated.'; end
end
