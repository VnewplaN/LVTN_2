# Physically interpretable buoyancy tuning

## Mass and centre of mass

- Main assembly: `350 kg`, CG properties in `usv.urdf.xacro`.
- Pump: `150 kg`; its uniform-volume STL centroid is approximately
  `(0.0012, -0.0005, -0.3762) m` in the pump frame.
- Fixed-link-lumped SDF result: mass `500.007 kg`, combined CG approximately
  `(0, 0, -0.4566) m` with `body_cg_z=-0.70 m`.
- `body_cg_y=-0.2698 m` balances the pump mounted at `y=+0.630 m`.
- `body_cg_z` is a named Xacro property and should ultimately be replaced by
  the measured CAD value.

The first trial at `body_cg_z=-0.15 m` produced a combined CG above the body
origin and capsized. `-0.50 m` still settled with about 12 degrees attitude.
The selected `-0.70 m`, together with the pump's actual STL centroid and
opposite cylinder orientation on the right pontoon, settled upright.

## Buoyancy geometry and math

Fresh-water density is `1000 kg/m^3`. Each sealed pontoon is represented by
three longitudinal cylinders, each with radius `0.25 m` and length
`1.9/3 = 0.633333 m`. Splitting improves pitch restoring-force resolution but
does not change total volume.

```text
V_one   = pi * 0.25^2 * 1.9 = 0.373064 m^3
V_total = 2 * V_one          = 0.746128 m^3
V_sub   = 500 / 1000         = 0.500000 m^3
fraction = V_sub / V_total   = 0.670126 = 67.013%
```

The right cylinders use yaw `pi`, matching the right visual. This matters in
Fortress graded-volume slicing: using the same tessellation direction on both
sides produced an approximately 8-degree residual attitude even with a
centred combined CG.

## Static-test configuration

The project is currently limited to spawning and validating the bare USV.
The pump is attached with a fixed joint. No controller package, ros2_control
description, controller manager, or pump command logic is included yet.

The installed Fortress hydrodynamics system supplies conservative linear and
quadratic damping. Added mass and Coriolis are disabled for this initial
hydrostatic model. The main tuneable terms are `zW` / `zWabsW` (heave),
`kP` / `kPabsP` (roll), and `mQ` / `mQabsQ` (pitch).

## Measured validation results

Tests used isolated Gazebo partitions, no thruster forces, wind, waves, or
current:

- Static: `z=0.2817 m`, roll `0.0007 deg`, pitch `0.0004 deg`.
- Initial roll 5 deg: returned to roll `0.0007 deg`.
- Initial pitch 5 deg: returned to pitch `0.0004 deg`.
- Initial heave -0.05 m: returned to `z=0.2817 m`.

## Commands

```bash
colcon build --packages-select usv_description --symlink-install
source install/setup.bash

# Static equilibrium
ros2 launch usv_description gazebo.launch.py

# Five-degree roll and pitch restoration
ros2 launch usv_description gazebo.launch.py spawn_roll:=0.0872665
ros2 launch usv_description gazebo.launch.py spawn_pitch:=0.0872665

# Start 0.05 m below measured equilibrium
ros2 launch usv_description gazebo.launch.py spawn_z:=0.2317
```

Close each Gazebo instance with `Ctrl+C` before starting the next test.
