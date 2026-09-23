# corobo — 4-Wheel GPS/Lidar Outdoor Robot with Object-Tracking Arm

ROS 2 **Humble** + **Gazebo Classic 11**. No `ros2_control` anywhere.

Includes:
- 4-wheel differential-drive base with GPS, IMU, and lidar
- GPS + IMU + wheel-odometry sensor fusion (robot_localization), output in lat/lon
- Nav2 navigation to a lat/lon goal, obstacle avoidance via lidar
- 2-DOF object-tracking arm (base rotation + elbow tilt), driven by Twist messages on
  `/cmd_vel_basejoint` and `/cmd_vel_elbowjoint` — velocity-controlled like real motors,
  not position servos. No collision geometry on the arm, so it never appears in `/scan`
  or the costmaps.
- Drone/object tracker (YOLO + webcam) publishing LEFT/RIGHT/UP/DOWN/STOP/LOST on
  `/drone_pos`, consumed by `arm_tracker_controller.py` to slew the arm toward the target.

---

## 1. Clone and build

```bash
sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins \
  ros-humble-xacro ros-humble-joint-state-publisher \
  ros-humble-robot-localization \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-geographic-msgs ros-humble-teleop-twist-keyboard \
  python3-serial git && \
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
source ~/.bashrc && \
cd ~ && \
git clone https://github.com/eshwareshanth-maker/corobo_gos_nav.git corobo_ws && \
cd ~/corobo_ws && \
chmod +x src/corobo/scripts/*.py && \
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/corobo_ws/install/corobo/lib/corobo && \
echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/corobo_ws/install/corobo/lib/corobo' >> ~/.bashrc && \
colcon build --symlink-install && \
source install/setup.bash && \
echo "source ~/corobo_ws/install/setup.bash" >> ~/.bashrc
```

For the object tracker (`drone_tracker.py`), also install its Python dependencies:
```bash
pip install ultralytics opencv-python --break-system-packages
```
Place your trained weights file at `src/corobo/scripts/yolov8n-drone.pt` (not included in
this repo — add your own model file there before running the tracker).

---

## 2. Verify the build

```bash
xacro src/corobo/urdf/corobo.urdf.xacro > /tmp/corobo.urdf && check_urdf /tmp/corobo.urdf
grep "datum" ~/corobo_ws/src/corobo/config/ekf.yaml
ls ~/corobo_ws/install/corobo/lib/corobo/libcorobo_arm_drive.so
```
- `check_urdf` should print the full link tree with no errors.
- `grep datum` should show `datum: [13.0100000, 80.2350000, 0.0]` and `wait_for_datum: true`.
- The `.so` file should exist — confirms the arm drive plugin compiled.

---

## 3. Start the simulation

```bash
cd ~/corobo_ws && source install/setup.bash
ros2 launch corobo bringup_sim.launch.py
```
Wait ~15–20 seconds for Gazebo, sensor fusion, and Nav2 to fully activate. In a new
terminal, confirm the GPS datum loaded:
```bash
cd ~/corobo_ws && source install/setup.bash
ros2 param get /navsat_transform datum
```

### Send a lat/lon navigation goal
```bash
ros2 run corobo latlon_goal.py --ros-args -p lat:=13.0102 -p lon:=80.2354
```

### Drive the base manually
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Drive the arm manually
```bash
python3 ~/corobo_ws/src/corobo/scripts/base_joint_driver.py --ros-args -p speed:=2.0
python3 ~/corobo_ws/src/corobo/scripts/elbow_joint_driver.py --ros-args -p speed:=1.5
```

---

## 4. Start the drone/object tracker + arm control

Two separate nodes, run in two terminals (order doesn't matter — they only communicate
over the `/drone_pos` topic).

**Terminal A — arm controller** (listens on `/drone_pos`, drives the arm motors):
```bash
cd ~/corobo_ws && source install/setup.bash
python3 ~/corobo_ws/src/corobo/scripts/arm_tracker_controller.py
```

**Terminal B — object tracker** (webcam + YOLO, publishes `/drone_pos`):
```bash
cd ~/corobo_ws && source install/setup.bash
python3 ~/corobo_ws/src/corobo/scripts/drone_tracker.py
```

Test the arm controller on its own without the tracker running:
```bash
ros2 topic pub --once /drone_pos std_msgs/msg/String "{data: 'LEFT'}"
ros2 topic pub --once /drone_pos std_msgs/msg/String "{data: 'UP'}"
ros2 topic pub --once /drone_pos std_msgs/msg/String "{data: 'STOP'}"
```

---

## 5. Hardware

See `arduino/corobo_base/corobo_base.ino` for the 4-wheel motor/encoder firmware, and
`scripts/serial_base.py` for the ROS 2 ↔ Arduino bridge. Launch on real hardware with:
```bash
ros2 launch corobo bringup_hw.launch.py port:=/dev/ttyACM0 gps_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1
```

---

## Repo layout

corobo/
├── urdf/corobo.urdf.xacro base links, joints, inertias
├── urdf/corobo.gazebo.xacro diff-drive, GPS, IMU, lidar, arm-drive plugins
├── urdf/arm.urdf.xacro 2-DOF arm (visual only, no collision -> invisible to lidar)
├── src/corobo_arm_drive.cpp Gazebo plugin: Twist-driven velocity motors for the arm
├── worlds/outdoor.world GPS datum + obstacles
├── config/ekf.yaml dual EKF + navsat_transform (pinned datum)
├── config/nav2_params.yaml Nav2, rolling costmaps, no static map
├── launch/ sim / localization / navigation / bringup launch files
├── scripts/latlon_goal.py lat/lon -> Nav2 goal
├── scripts/latlon_odom.py fused pose -> lat/lon
├── scripts/serial_base.py Arduino bridge (hardware)
├── scripts/base_joint_driver.py manual base-motor test driver
├── scripts/elbow_joint_driver.py manual elbow-motor test driver
├── scripts/arm_tracker_controller.py /drone_pos -> arm motor commands
├── scripts/drone_tracker.py YOLO + webcam object tracker -> /drone_pos
└── arduino/corobo_base/ 4-wheel base firmware
