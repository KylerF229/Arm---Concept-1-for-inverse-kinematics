function p = arm_resize_links(p,L1,L2)
% Rebuild masses for length changes using the existing uniform-tube model.
validateattributes([L1 L2],{'numeric'},{'real','finite','positive','numel',2});
p=arm_viewer_parameters(p); old=p; Lg=p.viewer.gripperLength;
p.L1=L1; p.L2=L2; p.Gripper=Lg; p.Ltotal=L1+L2+Lg;
if all(isfield(p,{'density','A','m_electronics'}))
    % Preserve any extra uniform link mass beyond the declared tube/wiring.
    oldTotal=old.L1+old.L2+Lg;
    oldWire=old.m_electronics*[old.L1 old.L2 Lg]/oldTotal;
    oldTube=old.density*old.A*[old.L1 old.L2 Lg];
    extra=[old.m1 old.m2]-oldTube(1:2)-oldWire(1:2);
    wire=p.m_electronics*[L1 L2 Lg]/p.Ltotal;
    tube=p.density*p.A*[L1 L2 Lg];
    p.m1_tube=tube(1); p.m2_tube=tube(2); p.m_gripper_tube=tube(3);
    p.m_wire1=wire(1); p.m_wire2=wire(2); p.m_wire_gripper=wire(3);
    p.m1=tube(1)+wire(1)+extra(1); p.m2=tube(2)+wire(2)+extra(2);
    deltaGripper=tube(3)+wire(3)-oldTube(3)-oldWire(3);
    if isfield(old,'m_gripper'), p.m_gripper=old.m_gripper+deltaGripper;
    else, p.m_gripper=old.viewer.gripperMass+deltaGripper; end
    p.viewer.gripperMass=old.viewer.gripperMass+deltaGripper;
else
    % Generic fallback: preserve each uniform link's kg/metre.
    p.m1=old.m1*L1/old.L1; p.m2=old.m2*L2/old.L2;
end
p.J1=p.m1*L1^2/12; p.J2=p.m2*L2^2/12;
assert(all([p.m1 p.m2 p.viewer.gripperMass]>=0),'Length change produced a negative mass.');
if isfield(p,'geometry')
    p.geometry.upperArm=L1; p.geometry.forearm=L2;
end
if isfield(p,'tool') && isfield(p.tool,'mass')
    p.tool.mass=p.tool.mass+p.viewer.gripperMass-old.viewer.gripperMass;
end
% Keep the project's three-joint visualization template synchronized.
if isfield(p,'models') && isscalar(p.models) && numel(p.models.joints)==3
    for k=1:2
        lengths=[L1 L2]; masses=[p.m1 p.m2];
        p.models.joints(k).linkVector=[lengths(k) 0 0];
        p.models.joints(k).com=[lengths(k)/2 0 0];
        p.models.joints(k).mass=masses(k);
    end
end
end
