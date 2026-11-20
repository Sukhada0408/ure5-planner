# Vision-Guided Grasping with UR5e and MoveIt2

A UR5e arm (mock hardware) moves to a pre-grasp pose above a coloured cube. The cube is located by a synthetic top-down camera and an OpenCV detector. The synthetic camera for rendering the cube and the detector are defined in a separate repo (cube_perception). The whole pipeline runs in Docker, on CPU only, with no physical robot.

This repo works in combination with the [cube_perception repo](https://github.com/Sukhada0408/cube_perception)

## Goal

Close the loop between perception and motion planning:

1. A synthetic camera renders a cube on a table.
2. A detector estimates the cube pose in `base_link`.
3. A C++ MoveIt2 client plans and executes a motion to a pose 15 cm above the cube, a virtual gripper then picks up the cube.

## Architecture

```
 synthetic_camera (Python)              detector (Python)
 renders cube, publishes                HSV threshold -> centroid ->
 Image + CameraInfo + GT pose  ───────► pinhole back-projection -> TF2 to base_link
 static TF base_link -> camera_link                    │
                                                       ▼ /object_pose (PoseStamped, base_link)
                                             grasp client (C++)
                                  trigger policy -> pre-grasp pose (+15 cm, tool down)
                                  -> MoveGroupInterface plan() -> execute()
                                                       │
                                                       ▼
                               MoveIt2 move_group  ──►  UR5e mock hardware controllers
```

Back-projection (top-down camera at known height, so no depth sensor is needed):

```
X = (u - cx) * Z / fx      Y = (v - cy) * Z / fy      Z = camera_height - object_height
```


## Quick start

Both workspaces are mounted into one container, and each process runs in its own terminal.

```bash
# 0. Start the container (both workspaces mounted)
docker run -it --rm --name ros2dev \
  -e DISPLAY=$DISPLAY -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg \
  -v ~/cube_perception:/home/ubuntu/ws_perception \
  -v ~/planner_client:/home/ubuntu/ws_planner \
  ros2-arm

Build (once, from each workspace root):

```bash
colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

Then in different terminals, run the following commands.

# Terminal 1: robot (mock hardware) and MoveIt
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_mock_hardware:=true launch_rviz:=false
# 2. Terminal 2:
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
# 3. Terminal 4 (source the planner workspace): grasp client
ros2 launch ur_planner_client planner_client.launch.py
# 4. Terminal 3 (source the perception workspace): camera and detector. Check the [cube_perception repo](https://github.com/Sukhada0408/cube_perception) for this
ros2 launch cube_perception perception.launch.py

```

In a params.yaml file, set the 'cube_x' and 'cube_y' parameters. Load the file (ros2 param load /synthetic_camera params.yaml) and watch the arm move to the grasp position.


## How the grasp client works

- **Trigger policy:** plans only when the cube moved more than `[1 cm]` since the last successful attempt, and never while a motion is running. A failed plan is retried on the next message.
- **Pre-grasp pose:** x, y from the detection, z raised by `[0.15 m]`, orientation fixed to a 180° rotation about x so the tool z-axis points down.
- **Frame guard:** targets not expressed in `base_link` are rejected with an error instead of silently sent to the wrong place.
- **Error handling:** the error code of both `plan()` and `execute()` is checked and logged.


## What I learned and debugged

- To set the target pose for planning and execution. The arguments passed to the functions and logging the subsequent errors.

- Loading two workspaces in a docker container. One workspace for cube_perception and one for planner_client.
