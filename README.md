# Arm task-pose viewer update

Copy the files in this ZIP into your arm project folder, replacing the viewer
files when prompted. Keep your existing arm_parameters.m, ARM_1.m,
Inverse_Dynamics.m and animation functions. They do not need replacement.
The project on OneDrive was read for integration; it was not modified here.

## Open the viewer

Run `start_arm_pose_study` to open it directly without the old target-selection
prompts or trajectory calculation. This starter copies the physical properties
from your current ARM_1.m and calls your current arm_parameters(p). If you
change the arm hardware later, update these properties in the starter too.

Your existing main-script call also works:

```matlab
viewer = plotArm3D([0 0 0],p); % three ABSOLUTE angles, radians
```

`p.plotOutputs` and `p.plot.model3D` both control whether it opens.
Your current arm_parameters is compatible: retain `p = arm_parameters(p)`.

## Study a task pose

1. Choose Ground pickup, Reach into drawer, or Custom target.
2. Edit X/Y/Z in metres and the tool angle in degrees. Press Enter or leave
   a field to commit it. The red target marker updates before the arm moves.
3. Press Solve target. Choose Closest pose, Higher elbow, or Lower elbow to
   choose the elbow configuration, then solve again.
4. Change the payload or move the relative joint sliders to inspect torques.
5. Give the pose a name and press Record pose. The current arm state, target,
   position error, absolute angles, payload and signed physical joint torques
   are added to `armPoseResults` in the MATLAB workspace.

The green dot is the actual TCP; the dashed connecting line shows its error
from the red target. The short red line indicates the requested tool direction.
Ground is Z=0. The rover dimensions and mount height come from your p.rover
fields unless explicitly overridden in p.viewer. Reset view restores a view
containing the ground, rover, target and current arm.

The drawer preset adds a translucent shelf reference below the target.
Its starting target is 0.20 m beyond the rover front at deck height, tool angle
0 degrees. Ground pickup starts 0.15 m beyond the front at Z=0.03 m, tool angle
-90 degrees. These are editable example tasks, not measured task requirements.
A target can remain visible even if it is unreachable; a failed solve leaves
both the current pose and its torque results unchanged.

Coordinates: X forward from the mount, Y left, Z above ground. The existing
planar arm moves only at Y=0. Nonzero-Y targets are displayed but rejected by
IK. Positive tool angle points upward; -90 degrees points down.
Home pose returns to the opening pose and keeps the target and current payload.

## Torque interpretation

The displayed values are signed static motor holding torques in N m. They
include m1/m2 at their midpoints, mM2 at the elbow, m3 at the wrist,
m_gripper at the gripper midpoint, and m_payload at the TCP. No p.tool or
p.models masses are added again. m3 already contains your wrist motors,
gripper actuator and camera; each mass is counted once.

The relative UI angles satisfy qAbsolute=cumsum(relativeAngles). Therefore
physical motor torques are H*Gabsolute, with H=[1 1 1;0 1 1;0 0 1], matching
ARM_1.m. Your CURRENT Inverse_Dynamics includes gripper and payload gravity
and matches this model with midpoint gripper COM and zero payload offset.
No safety factor is applied to the displayed raw holding torques.

Only static gravity is included. For a drawer-pulling force, contact with the
shelf, or accelerating a load, extra torque terms are required. The solver
checks geometric reach and relative joint limits; it does not certify ground,
rover or drawer clearance. Inspect the plotted configuration and use the
elbow branch control to explore alternate poses.

## Results and animation integration

```matlab
% After recording poses, inspect or save the table:
armPoseResults
writetable(armPoseResults,'arm_pose_results.csv');

% Existing trajectory animation: absolute radians, one row per time step.
if ~isempty(viewer) && isgraphics(viewer.figure)
    viewer.setPose(q_des(i,:)');
end

% Set a target and solve from code:
viewer.setTarget([0.50 0 0.03],-90);
success = viewer.solveTarget();
result = viewer.getResults();
torque = result.tauJoint;
```

Record pose stores the CURRENT arm pose even if you have not solved the target;
the position-error column helps distinguish those cases. Each viewer starts a
new record table; pressing Record pose publishes that viewer's table to
armPoseResults. Save a table before recording in a second viewer if needed.
Payload changes in this viewer do not rewrite p or your trajectory results.

## Validation

Run `test_arm_viewer` from your project folder. It covers target forward
kinematics, elbow selection, out-of-plane/unreachable targets, torque math,
current Inverse_Dynamics agreement, graphics updates, saved results and toggles.
This session could not start MATLAB because its preferences directory was not
writable. Independent numerical checks passed for both preset targets and
holding torques computed from mass lever arms; the updated MATLAB UI and
MATLAB tests still require execution on your machine.
