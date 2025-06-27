# ROS ArUco Robot Project

This workspace contains an example implementation for marker based
localisation and simple navigation.  The main package is
`aruco_localization` which provides three nodes:

* **camera_calibration_node.py** – interactively calibrate the camera using
  a chessboard pattern.
* **aruco_detector.py** – detect the four ArUco markers arranged as a
  2×2 grid on the floor and publish a 2‑D pose estimate of the robot.
* **motion_commander.py** – move the robot towards one of the markers
  using the pose estimate.

The original tutorial code is kept in the `QRfollowing` package.

## Usage

1. **Calibrate the camera**

   Run the calibration node and follow the on screen instructions.
   Capture several chessboard views with the `c` key and press `q` when
   finished.  The calibration results are written to `config/camera_info.yaml`.

   ```bash
   roslaunch aruco_localization calibration.launch
   ```

2. **Run detection and navigation**

   Launch the detector and controller together.  Set the desired goal
   marker ID with the `goal_id` parameter.

   ```bash
   roslaunch aruco_localization full_demo.launch goal_id:=3
   ```

During operation a window will display the camera feed with detected
markers outlined.  The controller publishes velocity commands at
`0.1 m/s` and after each small step waits for a new pose estimate from
`aruco_detector`.  If no update arrives the previous estimate is used
instead.

## Files

```
aruco_localization/
  scripts/     # Python nodes
  launch/      # Example launch files
  config/      # Camera calibration YAML
  maps/        # Marker map used for localisation
```

The markers are defined in `maps/map.yaml` as a 2×2 grid with one metre
spacing starting from marker `0` at the origin.
