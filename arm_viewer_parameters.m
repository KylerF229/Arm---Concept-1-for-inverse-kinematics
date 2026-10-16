function p = arm_viewer_parameters(p)
% Add viewer defaults to the existing planar-arm parameter structure.
if nargin==0, p=struct(); end
p=put(p,'L1',0.50); p=put(p,'L2',0.45); p=put(p,'g',9.81);
p=put(p,'m1',p.L1); p=put(p,'m2',p.L2); p=put(p,'mM2',0.50);
p=put(p,'m3',0.50); % Legacy mass at wrist; see README before adding tool mass.
p=put(p,'plotOutputs',true);
if ~isfield(p,'plot'), p.plot=struct(); end
p.plot=put(p.plot,'model3D',true);
if ~isfield(p,'viewer'), p.viewer=struct(); end
v=p.viewer;
v=put(v,'gripperLength',getOr(p,'Gripper',0.15));
v=put(v,'gripperMass',getOr(p,'m_gripper',0)); % Tube + its wiring, separate from m3.
v=put(v,'gripperCOM',v.gripperLength/2);
v=put(v,'payloadMass',getOr(p,'m_payload',5)); v=put(v,'payloadOffset',[0 0]); % local forward/up
v=put(v,'homeRelativeDeg',[35 65 -100]);
v=put(v,'limitsDeg',[-180 180; -180 180; -180 180]);
v=put(v,'baseXZ',[0 0]); % fixed carriage offset in legacy planar coordinates
r=getOr(p,'rover',struct());
v=put(v,'mountHeight',getOr(r,'mountHeight',0.60));
v=put(v,'mountBehindFront',getOr(r,'mountBehindFront',0.35));
v=put(v,'mountLeftOfCenter',getOr(r,'mountLeftOfCenter',0));
v=put(v,'deckHeight',getOr(r,'deckHeight',0.50));
v=put(v,'groundClearance',getOr(r,'groundClearance',0.15));
v=put(v,'roverLength',getOr(r,'length',1)); v=put(v,'roverWidth',getOr(r,'width',0.80));
v=put(v,'targetXYZ',[v.mountBehindFront+0.15 0 0.03]);
v=put(v,'targetAngleDeg',-90);
p.viewer=v;
end
function value=getOr(s,k,fallback)
if isfield(s,k), value=s.(k); else, value=fallback; end
end
function s=put(s,k,value)
if ~isfield(s,k), s.(k)=value; end
end
