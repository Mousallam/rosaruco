# ROS ArUco Robot Project

This repository contains a minimal example for localisation and waypoint
navigation using ArUco markers.  The `aruco_localization` package bundles a
set of Python nodes that calibrate a camera, detect markers on the floor and
command movement based on the detected pose.

## Package overview

The following executable scripts are located in `aruco_localization/scripts`:

* **camera_calibration_node.py** – interactively calibrate a webcam using a
  chessboard.  The resulting parameters are stored in
  `config/camera_info.yaml`.
* **aruco_detector.py** – detect markers, estimate the robot pose in 2‑D and
  publish it on `/robot_pose`.
* **motion_commander.py** – simple state machine that drives towards a marker
  and rotates to a desired final orientation.  By default velocity commands
  are published to `/turtle1/cmd_vel` so that the demo can be tried with
  `turtlesim`.
* **goal_publisher.py** – small helper that allows typing a goal marker ID and
  a final orientation on the command line.

The markers used for localisation are defined in `maps/map.yaml` as a 2×2 grid
with one metre spacing.

## Building the workspace

1. Create or use an existing catkin workspace and clone this repository into
   the `src` folder.
2. Install the required ROS packages (e.g. `rospy`, `cv_bridge`) and OpenCV for
   Python if not already present.
3. From the workspace root run:

   ```bash
   catkin_make
   source devel/setup.bash
   ```

## Calibration

Start the calibration node and follow the on‑screen instructions.  Press `c`
whenever the full chessboard is visible and `q` once enough samples have been
collected.

```bash
rosrun aruco_localization camera_calibration_node.py
```

The calibration data will be written to `config/camera_info.yaml`.

## Running the demo

A typical session consists of the following terminals:

1. **Start the core and turtlesim (for testing):**
   ```bash
   roscore &
   rosrun turtlesim turtlesim_node
   ```
2. **Run the marker detector:**
   ```bash
   rosrun aruco_localization aruco_detector.py
   ```
3. **Launch the motion commander:**
   ```bash
   rosrun aruco_localization motion_commander.py
   ```
4. **Publish goal markers interactively:**
   ```bash
   rosrun aruco_localization goal_publisher.py
   ```

Enter `<id> <angle_deg>` whenever prompted (e.g. `1 90`).  The turtle will
turn toward the specified marker, drive to it and finally rotate to the given
angle.

### Using a real robot

For a real robot simply change the output topic of `motion_commander.py` to
`/cmd_vel` or remap `/turtle1/cmd_vel` when launching the node.  All other
nodes remain the same.

```bash
rosrun aruco_localization motion_commander.py cmd_vel:=/cmd_vel
```

## Project files

```
aruco_localization/
  scripts/     # Python nodes described above
  config/      # Camera calibration YAML
  maps/        # Marker map for localisation
```
