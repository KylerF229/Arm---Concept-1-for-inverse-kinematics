# Optimize arm lengths for your targets

Open your viewer using ARM_1 or start_arm_pose_study as usual.

1. Enter a required target, tool angle and payload. Click **Record target / pose**.
2. Repeat for every required target. You can record unreachable targets: the
   optimizer uses the requested target, not the arm's current TCP position.
3. Click **Optimize lengths for recorded targets**.
4. Enter the minimum/maximum for L1 and L2 and the search step. The default is
   0.20 to 0.60 m for each link, in 0.01 m steps. These are editable search
   bounds, not a recommended hardware size. Existing lengths within the bounds
   are included so the grid cannot overlook the current design.
5. Keep **Shortest total length**, run the study, then **Apply best to viewer**.

Every recorded target is required, including its tool angle and payload.
L1 and L2 can differ; the gripper, mount and rover geometry stay fixed.
Both elbow solutions are checked against the relative joint limits. The best
clear elbow solution for each target minimizes its maximum absolute motor
holding torque. Among designs that reach all targets, the default objective
minimizes L1+L2, then worst holding torque to break ties. The optional torque
objective reverses those priorities.

The table ranks feasible candidates and reports the peak absolute holding
torque for each joint across all targets. The lower table gives signed torques
at each individual target for the selected design. These are raw static loads,
without safety factors, actuator ratings, acceleration or applied contact forces.

This is a finite search: the result is the shortest feasible design among
the tested combinations. Narrow the bounds around the leading result and use
a smaller step to refine it. Up to 10,000 combinations are allowed per run.
An unchanged design is a valid result if it is shortest among tested candidates.
The optimizer never discards a failed target to recommend a partial solution.

The default clearance check rejects links that cross the ground or an expanded
rover body box, using half p.Dtube (or 0.0127 m if not supplied) as radius.
This conservative geometric check excludes the fixed mount and does not model
drawer walls, payload volume, self-collision or motion between poses. Turning
it off gives reach-only feasibility and can admit poses through the rover.

## Updated parameters and saved results

Apply changes the lengths and masses in the current viewer and shows the most recently recorded
target's optimized pose. Old recorded torque rows retain their original lengths;
the optimizer reads only their target coordinates, orientation and payload.
New records include the currently applied lengths.

The MATLAB workspace contains:

- `armLengthStudy`: candidate ranking, original-design comparison, optimized
  poses, signed target torques, and the settings/targets used for the study.
- `pOptimized`: the applied geometry and rebuilt mass/inertia parameters.

Use these for later computations or to save the design:

```matlab
p = pOptimized;
save('optimized_arm_design.mat','pOptimized','armLengthStudy');
writetable(armLengthStudy.ranked,'arm_length_candidates.csv');
```

The hard-coded lengths in ARM_1 are not rewritten. Rerunning that script restores
its defaults. To reopen a saved optimized design:

```matlab
load('optimized_arm_design.mat','pOptimized');
viewer = plotArm3D([0 0 0],pOptimized);
```

Mass rebuilding follows your existing uniform tube and wiring model. Tube mass
scales with length; the fixed total electronics/wiring mass is redistributed
over all three segments. Additional declared uniform link mass is retained.
J1/J2 are recomputed about the midpoint. Wrist hardware and the provisional J3
stay fixed; geometry/template fields are synchronized. If tube properties are
absent, the fallback preserves each link's original mass per metre.

Programmatic use:

```matlab
opts = struct('L1Values',.30:.005:.45,'L2Values',.30:.005:.45, ...
              'objective','shortest','checkClearance',true);
result = viewer.optimizeLengths(opts);
if ~isempty(result.bestP)
    viewer.applyLengthStudy(result);
end
```

Run `test_arm_length_study` for mass/inertia, analytical reach, shortest-candidate
ranking, no-solution behavior, basic clearance, recording and viewer application
checks. The current implementation has been exercised in MATLAB R2025b.

## New unreachable targets and automatic expansion

Record a new target even when Solve target reports it unreachable. The open
length-study window reads the latest recorded targets each time you run it.
If you click Apply after recording another target, it reruns first so it cannot
apply a design based on an old target list. Previously recorded targets remain
required; the new target is added to them.

Auto-expand is on by default. If no design in the initial range meets all
recorded targets, both maximum lengths grow in 0.20 m increments up to the
editable Maximum per link ceiling (default 1.00 m). Minimum lengths and grid
step stay unchanged. Expansion stops at the first range containing a feasible
design, or at the ceiling. The chosen design is the shortest sampled design in
that searched range; it is not a global optimum across all possible lengths.

Apply updates the geometry and displays the most recently recorded target.
If there is still no feasible design, it leaves the current arm unchanged.
Targets outside Y=0, joint-limit restrictions, and collisions may remain
infeasible regardless of length. Search expansion does not relax those checks.
For large ranges or finer steps, the 10,000-candidate limit may require increasing
the step. Turn off Auto-expand when the entered upper bounds are hard limits.

The programmatic viewer.optimizeLengths method also enables expansion by
default. Supply autoExpand=false to keep its bounds fixed, or maxLinkLength
and expansionIncrement to configure expansion. The original low-level
arm_optimize_lengths function remains a fixed-grid search.