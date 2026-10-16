function viewer = plotArm3D(q,p)
% Compatible with plotArm3D([q1 q2 q3],p). ABSOLUTE angles in radians.
% Returns viewer.setPose(q) for updates from an existing animation loop.
p=arm_viewer_parameters(p);
viewer=[];
if ~p.plotOutputs || ~p.plot.model3D, return; end
q=q(:);
assert(numel(q)==3 && all(isfinite(q)),'Expected three absolute angles in radians.');
% Opening pose becomes the Home pose, while sliders display relative angles.
p.viewer.homeRelativeDeg=[q(1); diff(q)]'*180/pi;
assert(all(p.viewer.homeRelativeDeg'>=p.viewer.limitsDeg(:,1) & ...
           p.viewer.homeRelativeDeg'<=p.viewer.limitsDeg(:,2)), ...
       'Initial pose exceeds p.viewer.limitsDeg.');
viewer=arm_pose_viewer(p);
end
