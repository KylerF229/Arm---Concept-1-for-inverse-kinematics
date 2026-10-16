function api = arm_pose_viewer(p)
% Interactive task poses; API pose angles are ABSOLUTE radians.
if nargin==0, p=struct(); end
p=arm_viewer_parameters(p); api=[];
if ~p.plotOutputs || ~p.plot.model3D, return; end
v=p.viewer; rel=v.homeRelativeDeg(:); payload=v.payloadMass;
xyz=v.targetXYZ(:)'; angle=v.targetAngleDeg; records=table();
f=figure('Name','Arm task poses and holding torques','NumberTitle','off', ...
 'Color','w','Position',[60 50 1240 850]);
ax=axes(f,'Position',[.065 .35 .60 .59]); hold(ax,'on'); grid(ax,'on');
xlabel(ax,'X forward from mount (m)'); ylabel(ax,'Y left (m)'); zlabel(ax,'Z above ground (m)');
title(ax,'Rover, arm and task target'); axis(ax,'equal');
R=p.L1+p.L2+v.gripperLength;
xback=v.mountBehindFront-v.roverLength; yside=-v.roverWidth/2-v.mountLeftOfCenter;
ground=patch(ax,'XData',[-1 1 1 -1],'YData',[-1 -1 1 1], ...
 'ZData',[0 0 0 0],'FaceColor',[.86 .83 .75],'FaceAlpha',.55,'EdgeColor','none');
box3(ax,[xback yside v.groundClearance], ...
 [v.roverLength v.roverWidth max(.02,v.deckHeight-v.groundClearance)],[.4 .47 .54],.28);
text(ax,xback,yside,v.deckHeight+.04,'Rover deck');
plot3(ax,[0 v.baseXZ(1)],[0 0],[v.deckHeight v.mountHeight+v.baseXZ(2)],'k-','LineWidth',5);
arm=plot3(ax,nan,nan,nan,'-','LineWidth',5, ...
 'Color',[.1 .33 .5]);
joints=plot3(ax,nan,nan,nan,'o','LineStyle','none','MarkerSize',11, ...
 'MarkerFaceColor',[1 .48 .08],'MarkerEdgeColor',[.65 .26 .02],'LineWidth',1);
tcp=plot3(ax,nan,nan,nan,'o','MarkerFaceColor',[.1 .7 .3],'MarkerEdgeColor','k','MarkerSize',9);
target=plot3(ax,nan,nan,nan,'rp','MarkerFaceColor',[1 .25 .2],'MarkerSize',14);
drop=plot3(ax,nan,nan,nan,'r:'); aim=plot3(ax,nan,nan,nan,'r-','LineWidth',2);
errline=plot3(ax,nan,nan,nan,'--','Color',[.65 .2 .2]);
targetLabel=text(ax,0,0,0,'');
drawer=box3(ax,[0 0 0],[.25 .32 .02],[.2 .6 .85],.25); set(drawer,'Visible','off');
ta=axes(f,'Position',[.085 .085 .55 .18]); bars=bar(ta,1:3,zeros(3,1));
grid(ta,'on'); xticks(ta,1:3); xticklabels(ta,{'J1','J2','J3'}); ylabel(ta,'Holding torque (N m)');
panel=uipanel(f,'Title','Task pose controls','Position',[.70 .035 .29 .93]);
label([.04 .94 .92 .035],'Task starting point (editable)');
preset=uicontrol(panel,'Style','popupmenu','Units','normalized','Position',[.04 .895 .92 .045], ...
 'String',{'Custom target','Ground pickup','Reach into drawer'},'Callback',@choosePreset);
label([.04 .855 .92 .03],'Target: X forward / Y left / Z above ground (m)');
ed=gobjects(3,1);
for k=1:3
 ed(k)=uicontrol(panel,'Style','edit','Units','normalized','Position',[.04+(k-1)*.315 .81 .29 .045], ...
 'String',num2str(xyz(k)),'Callback',@editTarget);
end
label([.04 .758 .64 .035],'Tool angle: 0 forward, -90 down');
phi=uicontrol(panel,'Style','edit','Units','normalized','Position',[.73 .752 .23 .045], ...
 'String',num2str(angle),'Callback',@editTarget);
button([.04 .69 .44 .05],'Solve target',@solve);
button([.52 .69 .44 .05],'Reset view',@resetView);
label([.04 .642 .47 .035],'Relative joints (degrees)');
branch=uicontrol(panel,'Style','popupmenu','Units','normalized','Position',[.52 .633 .44 .045], ...
 'String',{'Closest pose','Higher elbow','Lower elbow'});
sl=gobjects(3,1); val=gobjects(3,1);
for k=1:3
 y=.59-(k-1)*.06; label([.04 y .1 .035],sprintf('J%d',k));
 sl(k)=uicontrol(panel,'Style','slider','Units','normalized','Position',[.16 y .6 .035], ...
 'Min',v.limitsDeg(k,1),'Max',v.limitsDeg(k,2),'Value',rel(k),'Callback',@move);
 val(k)=label([.79 y .19 .035],'');
end
label([.04 .405 .55 .04],'Payload mass (kg)');
massEdit=uicontrol(panel,'Style','edit','Units','normalized','Position',[.66 .405 .3 .04], ...
 'String',num2str(payload),'Callback',@massChanged);
readout=label([.04 .26 .92 .135],'');
nameEdit=uicontrol(panel,'Style','edit','Units','normalized','Position',[.04 .215 .92 .04], ...
 'String','Pose 1');
button([.04 .155 .44 .05],'Record target / pose',@recordPose);
button([.52 .155 .44 .05],'Home pose',@home);
button([.04 .10 .92 .045],'Optimize lengths for recorded targets',@openLengthStudy);
status=label([.04 .043 .92 .055],'Edit the red target, then solve.');
label([.04 .004 .92 .04],'Static gravity only; no contact-force or collision check.');
api=struct('figure',f,'setPose',@setPose,'getPose',@getPose, ...
 'setTarget',@setTarget,'solveTarget',@solveTarget,'getResults',@getResults, ...
 'recordPose',@recordPose,'getRecords',@getRecords,'getParameters',@getParameters, ...
 'optimizeLengths',@optimizeLengths,'applyLengthStudy',@applyLengthStudy);
drawTarget(); refresh(); resetView();
 function h=label(pos,str)
  h=uicontrol(panel,'Style','text','Units','normalized','Position',pos,'String',str, ...
   'HorizontalAlignment','left','BackgroundColor',get(panel,'BackgroundColor'));
 end
 function button(pos,str,cb)
  uicontrol(panel,'Style','pushbutton','Units','normalized','Position',pos,'String',str,'Callback',cb);
 end
 function q=getPose(), q=cumsum(rel)*pi/180; end
 function s=getResults(), s=arm_static_pose(getPose(),p,payload); end
 function t=getRecords(), t=records; end
 function params=getParameters(), params=p; end
 function result=optimizeLengths(options)
  if nargin==0, options=struct(); end
  result=arm_optimize_recorded_targets(p,records,options);
 end
 function openLengthStudy(~,~)
  if isempty(records), set(status,'String','Record each required target first, then optimize lengths.'); return; end
  arm_length_study(p,@getRecords,@applyLengthStudy);
 end
 function applyLengthStudy(result)
  if ~isgraphics(f), error('The original viewer has been closed.'); end
  assert(~isempty(result.bestP),'No feasible length design to apply.');
  p=result.bestP; v=p.viewer; R=p.L1+p.L2+v.gripperLength;
  selected=height(result.targets); % Show the newly added target after resizing.
  rel=[result.poses(selected,1);diff(result.poses(selected,:)')]*180/pi;
  v.homeRelativeDeg=rel'; p.viewer.homeRelativeDeg=rel';
  payload=result.targets.Payload_kg(selected); p.m_payload=payload;
  v.payloadMass=payload; p.viewer=v; set(massEdit,'String',num2str(payload));
  setTarget([result.targets.TargetX_m(selected) result.targets.TargetY_m(selected) result.targets.TargetZ_m(selected)], ...
   result.targets.TargetAngle_deg(selected));
  assignin('base','pOptimized',p);
  set(status,'String',sprintf('Applied L1 %.3f m / L2 %.3f m. Workspace: pOptimized.',p.L1,p.L2));
 end
 function move(~,~), rel=arrayfun(@(h)get(h,'Value'),sl); refresh(); end
 function home(~,~), rel=v.homeRelativeDeg(:); refresh(); end
 function setPose(q)
  if ~isgraphics(f), return; end
  q=q(:); assert(numel(q)==3 && all(isfinite(q)),'Expected 3 absolute angles.');
  r=[q(1);diff(q)]*180/pi;
  assert(all(r>=v.limitsDeg(:,1)-1e-9 & r<=v.limitsDeg(:,2)+1e-9),'Pose exceeds relative limits.');
  rel=r; refresh();
 end
 function massChanged(~,~)
  m=str2double(get(massEdit,'String'));
  if ~isfinite(m)||m<0, set(massEdit,'String',num2str(payload)); return; end
  payload=m; refresh();
 end
 function setTarget(position,toolAngleDeg)
  position=position(:)';
  assert(numel(position)==3 && all(isfinite(position)) && isscalar(toolAngleDeg) && isfinite(toolAngleDeg), ...
   'Expected finite XYZ and angle in degrees.');
  xyz=position; angle=toolAngleDeg;
  for j=1:3, set(ed(j),'String',num2str(xyz(j))); end
  set(phi,'String',num2str(angle)); drawTarget(); refresh(); resetView();
  set(status,'String','Target updated. Press Solve target to move the arm.');
 end
 function valid=editTarget(varargin)
  position=arrayfun(@(h)str2double(get(h,'String')),ed)'; a=str2double(get(phi,'String'));
  valid=all(isfinite([position a]));
  if ~valid, set(status,'String','Enter finite target values.'); return; end
  setTarget(position,a);
 end
 function drawTarget()
  set(target,'XData',xyz(1),'YData',xyz(2),'ZData',xyz(3));
  set(drop,'XData',[xyz(1) xyz(1)],'YData',[xyz(2) xyz(2)],'ZData',[0 xyz(3)]);
  tail=xyz-.12*[cosd(angle) 0 sind(angle)];
  set(aim,'XData',[tail(1) xyz(1)],'YData',[tail(2) xyz(2)],'ZData',[tail(3) xyz(3)]);
  set(targetLabel,'Position',xyz+[.02 0 .04],'String',sprintf('Target (%.2f, %.2f, %.2f)',xyz));
  if get(preset,'Value')==3
   V=cuboidVertices([xyz(1)-.12 xyz(2)-.16 xyz(3)-.04],[.25 .32 .02]);
   set(drawer,'Vertices',V,'Visible','on');
  else, set(drawer,'Visible','off'); end
 end
 function choosePreset(~,~)
  switch get(preset,'Value')
   case 2, set(nameEdit,'String','Ground pickup'); setTarget([v.mountBehindFront+.15 0 .03],-90);
   case 3, set(nameEdit,'String','Drawer reach'); setTarget([v.mountBehindFront+.20 0 v.deckHeight],0);
   otherwise, set(drawer,'Visible','off');
  end
 end
 function solve(~,~)
  if editTarget(), solveTarget(); end
 end
 function success=solveTarget()
  [q,msg]=arm_target_pose(xyz,angle,getPose(),p,get(branch,'Value')); success=~isempty(q);
  if success, setPose(q); resetView(); end
  set(status,'String',msg);
 end
 function refresh()
  s=getResults(); P=s.pointsXYZ;
  set(arm,'XData',P(:,1),'YData',P(:,2),'ZData',P(:,3));
  set(joints,'XData',P(1:3,1),'YData',P(1:3,2),'ZData',P(1:3,3));
  set(tcp,'XData',P(4,1),'YData',P(4,2),'ZData',P(4,3));
  set(errline,'XData',[P(4,1) xyz(1)],'YData',[P(4,2) xyz(2)],'ZData',[P(4,3) xyz(3)]);
  for j=1:3, set(sl(j),'Value',rel(j)); set(val(j),'String',sprintf('%.1f',rel(j))); end
  set(bars,'YData',s.tauJoint); ylim(ta,[-1 1]*max(1,1.2*max(abs(s.tauJoint))));
  title(ta,sprintf('Holding torques | %.2f kg | L1 %.3f / L2 %.3f m',payload,p.L1,p.L2));
  angleError=mod(sum(rel)-angle+180,360)-180;
  set(readout,'String',sprintf(['TCP: X %.3f  Y %.3f  Z %.3f m\n' ...
   'Target error: %.1f mm / %.1f deg\nJ1: %+.2f   J2: %+.2f   J3: %+.2f N m\nMotion plane: Y = 0'], ...
   s.tcpXYZ,1000*norm(s.tcpXYZ-xyz),angleError,s.tauJoint));
  drawnow limitrate;
 end
 function resetView(varargin)
  P=getResults(); P=P.pointsXYZ;
  xmin=min([xback-.15 v.baseXZ(1)-.2 xyz(1)-.2 P(:,1)']);
  xmax=max([v.mountBehindFront+.3 v.baseXZ(1)+R+.1 xyz(1)+.2 P(:,1)']);
  ymin=min([yside-.2 xyz(2)-.2]); ymax=max([yside+v.roverWidth+.2 xyz(2)+.2]);
  zmin=min([-.04 xyz(3)-.08 P(:,3)']); zmax=max([v.mountHeight+.35 xyz(3)+.2 P(:,3)'+.1]);
  xlim(ax,[xmin xmax]); ylim(ax,[ymin ymax]); zlim(ax,[zmin zmax]);
  set(ground,'XData',[xmin xmax xmax xmin],'YData',[ymin ymin ymax ymax]);
  view(ax,38,23); axis(ax,'equal');
 end
 function recordPose(varargin)
  s=getResults(); q=getPose()*180/pi;
  names={'TargetX_m','TargetY_m','TargetZ_m','TargetAngle_deg','TCPX_m','TCPY_m','TCPZ_m', ...
   'q1Abs_deg','q2Abs_deg','q3Abs_deg','Payload_kg','J1_Nm','J2_Nm','J3_Nm','PositionError_m'};
  row=array2table([xyz angle s.tcpXYZ q' payload s.tauJoint' norm(s.tcpXYZ-xyz)],'VariableNames',names);
  row.PoseName=string(get(nameEdit,'String')); row.L1_m=p.L1; row.L2_m=p.L2;
  records=[records;row];
  assignin('base','armPoseResults',records);
  set(status,'String',sprintf('Recorded current pose (%d total). Table: armPoseResults in MATLAB workspace.',height(records)));
 end
end
function h=box3(ax,o,d,color,alpha)
h=patch(ax,'Vertices',cuboidVertices(o,d),'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8], ...
 'FaceColor',color,'FaceAlpha',alpha,'EdgeColor',color);
end
function V=cuboidVertices(o,d)
V=o+[0 0 0;d(1) 0 0;d(1) d(2) 0;0 d(2) 0;0 0 d(3);d(1) 0 d(3);d;0 d(2) d(3)];
end
