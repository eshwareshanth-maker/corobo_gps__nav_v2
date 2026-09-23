# corobo_gos_nav
the robot which uses autonomus gps navigation

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

## 1. Clone the repo

```bash
mkdir -p ~/corobo_ws
cd ~
git clone https://github.com/eshwareshanth-maker/corobo_gps__nav_v2.git corobo_ws
```

---

## 2. Install ROS 2 + simulation dependencies

```bash
sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins \
  ros-humble-xacro ros-humble-joint-state-publisher \
  ros-humble-robot-localization \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-geographic-msgs ros-humble-teleop-twist-keyboard \
  python3-serial git

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Only needed if you'll run the real hardware from this machine:
```bash
sudo apt install -y ros-humble-nmea-navsat-driver ros-humble-rplidar-ros
```

---

## 3. Build the workspace

```bash
cd ~/corobo_ws
chmod +x src/corobo/scripts/*.py

# so Gazebo can find the custom arm-drive plugin
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/corobo_ws/install/corobo/lib/corobo
echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/corobo_ws/install/corobo/lib/corobo' >> ~/.bashrc

colcon build --symlink-install
source install/setup.bash
echo "source ~/corobo_ws/install/setup.bash" >> ~/.bashrc
```

---

## 4. Verify the build

```bash
xacro src/corobo/urdf/corobo.urdf.xacro > /tmp/corobo.urdf && check_urdf /tmp/corobo.urdf
grep "datum" ~/corobo_ws/src/corobo/config/ekf.yaml
ls ~/corobo_ws/install/corobo/lib/corobo/libcorobo_arm_drive.so
```

- `check_urdf` should print the full link tree (base_footprint → base_link → wheels, lidar,
  imu, gps, arm links) with no errors.
- `grep datum` should show `datum: [13.0100000, 80.2350000, 0.0]` and `wait_for_datum: true`.
- The `.so` file should exist — confirms the arm drive plugin compiled.

---

## 5. Start the simulation

```bash
cd ~/corobo_ws && source install/setup.bash
ros2 launch corobo bringup_sim.launch.py
```

Wait ~15–20 seconds for Gazebo, sensor fusion, and Nav2 to fully activate.

In a **new terminal**, confirm the GPS datum loaded:
```bash
cd ~/corobo_ws && source install/setup.bash
ros2 param get /navsat_transform datum
```

### Send a lat/lon navigation goal
```bash
ros2 run corobo latlon_goal.py --ros-args -p lat:=13.0102 -p lon:=80.2354
```

Watch it drive there in Gazebo, planning around obstacles using the lidar-fed costmap.

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

## 6. Object tracking + arm control

Two separate nodes, run in two terminals (order doesn't matter — they only communicate
over the `/drone_pos` topic: `LEFT`/`RIGHT`/`UP`/`DOWN`/`STOP`/`LOST`).

### 6a. Install the tracker's Python dependencies

The tracker uses YOLO + OpenCV, which are not part of the ROS install. Create a
virtual environment for it (keep this out of the repo — it's machine-specific):
```bash
cd ~/corobo_ws
python3 -m venv drone_tracker_n/n_env
source drone_tracker_n/n_env/bin/activate
pip install ultralytics opencv-python numpy
```

Place your trained YOLO weights at:

~/corobo_ws/drone_tracker_n/yolov8n-drone.pt
