# /betabot/description

This folder contains the URDF and mesh files which describe BetaBot hardware
to be used by ROS.

## Robot dimensions

The robot geometry is derived from the Polycam 3D scan `meshes/betabot_scan.glb`
(LiDAR photogrammetry, exported by Polycam at metric scale; glTF Y-up axes,
robot front pointing towards -Z of the scan).

The scale is validated against known LEGO MINDSTORMS part sizes:

- the head is an NXT brick (108.4 mm long): the scan measures 115.5 mm along
  the same axis (~6% thicker, consistent with photogrammetric surface noise),
- the drive treads touch the ground with a 25 mm radius, matching
  `tread_radius = 0.025` in `nxt_controllers`.

Scanned robot: **417 mm long (X) x 273 mm wide (Y) x 486 mm high (Z)**.

Scan axes to ROS axes (X forward, Y left, Z up):

    X_ros = -Z_scan, Y_ros = -X_scan, Z_ros = Y_scan

See `dimensions.json` for the measured values and per-link mesh bounds.

## Contents

- `urdf/robot_description.urdf` : robot description (ROS conventions, X
  forward, Y left, Z up). `base_link` sits on the drive axle (25 mm above the
  ground, matching `tread_radius = 0.025` in `nxt_controllers`),
  `base_footprint` on the ground. Joint and frame names match the existing
  ROS code:
  - `left_tread`, `right_tread` (NXT1 PORT_A/PORT_C)
  - `torso_joint` (NXT1 PORT_B), `head_joint`/`laser_joint`/`arms_joint` (NXT2)
  - sensor frames: `left/right/back_bumper_link`, `ultrasonic_sensor_link`,
    `sound_sensor_link`, `color_sensor_link`, `line_following_sensor_link`,
    `rfid_sensor_link`, `usb_cam_link`, `base_laser` (LD06), `imu_link` (OpenCR).
- `meshes/betabot_{base,torso,head,arms,full}.stl` : visual meshes segmented
  from the Polycam scan, in metres, each one expressed in its URDF link frame
  (anchored on the joint origins listed in `dimensions.json`).
- `meshes/betabot_scan.glb` : source Polycam scan (textured glTF binary).
- `meshes/lego model.io` : legacy BrickLink Studio draft model (kept for
  reference only, superseded by the scan).
- `generate_meshes.py` : regenerates the STL meshes and `dimensions.json`
  from the Polycam scan (pure standard library, no ROS required):

```bash
python3 generate_meshes.py
```

- `dimensions.json` : measured dimensions (bounding box, joint origins,
  scan-to-ROS frame mapping, mesh stats).

## Notes

- The LiDAR frame is `base_laser`, as published by the `ldlidar_stl` driver in
  `core.launch`; the URDF chains it to the motorised `laser_joint` mount.
- The scanned surface is an open photogrammetry mesh (holes, ~50k faces
  total); it is used for visuals only. Collision geometries are simple
  boxes/cylinders sized from the scan bounding boxes.
- Link segmentation is height-based on the scan vertical axis (base < 0.13 m,
  torso 0.13-0.26 m, arms/shoulders 0.26-0.36 m, head >= 0.36 m); the head
  appears tilted forward on the scan, as built.
