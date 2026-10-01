# APF Auto Planner

Open `index.html` directly in a browser. The planner is dependency-free.

- Set the robot length and width, then click to add route poses. Each pose is displayed as a
  correctly scaled robot footprint rather than a point.
- The first route pose is the autonomous start pose. Its exact coordinates and heading are shown
  in the planner and generated into the routine's odometry reset.
- Drag a footprint to reposition it. Drag its orange heading handle, or use the selected-robot
  heading field, to set its angle.
- Shift-click or use **Add obstacle** to add obstacle rectangles; drag them on the field.
- Click a route segment to select its intention and tuning values.
- **Optimize curved route** uses a shortest-path visibility-graph optimizer between the points.
- The optimizer expands obstacles by the robot footprint plus a safety margin, rejects every route
  segment that intersects that clearance area, rounds corners only when the resulting curve remains
  clear, and samples the result for path following.
- Enable **Face direction of travel** to set each moving pose's heading from its next path segment;
  the final pose retains the heading you explicitly set for it.
- **Generate Java** produces a complete class for `src/main/java/frc/robot/auto/routines/`.
  Add its `build()` call to `AutoSelector` to make the new routine selectable.

The planner renders `FE-2026-_REBUILT_Playing_Field_With_Fuel_With_Background.png`, the same image
used by the field tools. The complete image maps to the project field dimensions (`16.540988 m ×
8.069326 m`), with WPILib's blue-alliance origin at its bottom-left. Generated coordinates are in
meters and can be used directly in `Pose2d`.
