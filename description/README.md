# /betabot/description

This folder contains the URDF and mesh files which describe BetaBot hardware to be used by ROS.

## Robot dimensions

The robot geometry is derived from the BrickLink Studio model `meshes/lego model.io`
(LEGO CAD archive of the robot). The LDraw scale is used as the physical reference:

- 1 LDU (LDraw Unit) = 0.4 mm
- 1 stud = 20 LDU = 8 mm (LEGO Technic / MINDSTORMS grid)

The scale was validated against the Technic axles present in the model
(Axle 2 = 40 LDU = 16 mm ... Axle 12 = 240 LDU = 96 mm, all exact) and against
official MINDSTORMS NXT part sizes (NXT brick 53788, NXT motor 53787,
ultrasonic sensor 40490, touch sensor 53793, light sensor 55969).

Assembled robot (263 parts): **374 mm long x 133 mm wide x 446 mm high**.

See `dimensions.json` for the measured values and per-link mesh bounds.

## Contents

- `urdf/robot_description.urdf` : robot description (ROS conventions, X forward,
  Y left, Z up). `base_link` sits on the drive axle (25 mm above the ground,
  matching `tread_radius = 0.025` in `nxt_controllers`), `base_footprint` on the
  ground. Joint and frame names match the existing ROS code:
  - `left_tread`, `right_tread` (NXT1 PORT_A/PORT_C)
  - `torso_joint` (NXT1 PORT_B), `head_joint`/`laser_joint`/`arms_joint` (NXT2)
  - sensor frames: `left/right/back_bumper_link`, `ultrasonic_sensor_link`,
    `sound_sensor_link`, `color_sensor_link`, `line_following_sensor_link`,
    `rfid_sensor_link`, `usb_cam_link`, `base_laser` (LD06), `imu_link` (OpenCR).
- `meshes/betabot_{base,torso,head,arms,full}.stl` : visual meshes generated
  from the LEGO model, in metres, each one expressed in its URDF link frame.
- `meshes/lego model.io` : source BrickLink Studio model (password-protected
  ZIP, password `soho0909`, contains the LDraw MPD `model2.ldr`).
- `generate_meshes.py` : regenerates the STL meshes and `dimensions.json`
  from the LEGO model (pure standard library, no ROS required):

```bash
python3 generate_meshes.py
```

- `dimensions.json` : measured dimensions (bounding box, joint origins, mesh stats).

## Notes

- The LiDAR frame is `base_laser`, as published by the `ldlidar_stl` driver in
  `core.launch`; the URDF chains it to the motorised `laser_joint` mount.
- Mesh decimation is voxel-based (2.5 mm, 4 mm for the dense torso), keeping
  each STL under ~1.3 MB for RViz.
