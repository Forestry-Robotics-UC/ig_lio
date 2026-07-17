# FRUC iG-LIO (ROS 2)

## Citation

If you use this ROS 2 port in your work, please cite:

```bibtex
@article{carvalho2026numerically,
  title={A Numerically-Robust ROS 2 Port of iG-LIO: Diagnosing and Fixing Toolchain-Induced Failures in Incremental GICP LiDAR-Inertial Odometry},
  author={Carvalho, Afonso E and Portugal, David and Peixoto, Paulo},
  journal={arXiv preprint arXiv:2607.09947},
  year={2026}
}
```

---

A ROS 2 port of iG-LIO, a tightly-coupled LiDAR-inertial odometry system. This
fork preserves the functionality of the original code, but introduces some
important modifications:

- Ports the package from ROS 1 to **ROS 2 (Jazzy)** — `ament_cmake`, `rclcpp`,
  launch files and parameters in the ROS 2 style;
- Makes the **Livox** dependency optional instead of mandatory: the driver is no
  longer required to build, but Livox support is still compiled in when
  `livox_ros_driver2` is present (see [Sensor support](#sensor-support));
- Adds support for the legacy Velodyne Velarray M1600 LiDAR (by Pedro Tomás);
- Fixes the point cloud undistorting mechanism (at least with the Ouster) by
  updating Ouster-related code to the new message field names, types and other
  small details;
- Fixes solver NaN/divergence issues introduced by the port (QoS depth and an
  oneTBB + Eigen `parallel_reduce` incompatibility shipped with current ROS 2);
- Makes the subscription **QoS** and the **TF frame names** configurable from
  the YAML config, so the node drops into different bags/setups without code
  changes;
- Adds a well-tuned configuration file for the Ouster OS1 Rev7 128-channel
  LiDAR + internal IMU.

### Acknowledgement

The original repository can be found [here](https://github.com/zijiechenrobotics/ig_lio).

---

## Build

This is a standard `colcon` package. From your workspace root:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y   # install dependencies
colcon build --packages-select ig_lio
source install/setup.bash
```

❗ **Note**: Change `~/ros2_ws` to your actual workspace path.

Tested on **ROS 2 Jazzy**.

---

## Usage

Using the Ouster OS1 Rev7 128-channel LiDAR + internal IMU as the example:

```bash
ros2 launch ig_lio lio_ouster.launch.py
```

with either the LiDAR driver running in parallel (online) or a bag playing back
(offline). The launch file sets `use_sim_time: true`, so for offline playback
remember to play the bag with `--clock`:

```bash
ros2 bag play <your_bag> --clock
```

On startup the node subscribes to the LiDAR and IMU topics and begins publishing
the `odom -> base_link` transform on `/tf`, plus the odometry and cloud topics
listed below.

### Topics

Subscribed (topic names come from the config):

| Topic            | Type                          | Notes                       |
|------------------|-------------------------------|-----------------------------|
| `lidar_topic`    | `sensor_msgs/PointCloud2`     | raw scan                    |
| `imu_topic`      | `sensor_msgs/Imu`             | IMU                         |

Published:

| Topic           | Type                       | Notes                                   |
|-----------------|----------------------------|-----------------------------------------|
| `/lio_odom`     | `nav_msgs/Odometry`        | pose + real computed covariance         |
| `current_scan`  | `sensor_msgs/PointCloud2`  | dense scan in the `odom_frame`          |
| `keyframe_scan` | `sensor_msgs/PointCloud2`  | downsampled keyframe scan               |
| `/path`         | `nav_msgs/Path`            | trajectory (latched / transient-local)  |
| `/tf`           | —                          | `odom_frame -> base_frame`              |

---

## Sensor support

This port keeps the original sensor coverage, but with honest confidence levels.
Only the Ouster path has been validated on hardware; the others were ported but
not tested, so treat them as starting points and verify before relying on them.

| `lidar_type` | Sensor(s)              | Message type                       | Status                                   |
|--------------|------------------------|------------------------------------|------------------------------------------|
| `ouster`     | Ouster OS1-128 (et al.)| `sensor_msgs/PointCloud2`          | ✅ Tested & tuned (`config/ouster128.yaml`)|
| `velodyne`   | Velodyne (VLP etc.)    | `sensor_msgs/PointCloud2`          | ⚠️ Ported, untested                       |
| `Hesai`      | Hesai                  | `sensor_msgs/PointCloud2`          | ⚠️ Ported, untested                       |
| `M1600`      | Velodyne Velarray M1600| `sensor_msgs/PointCloud2`          | ⚠️ Ported, untested                       |
| `livox_points`| Livox as PointCloud2 (e.g. Mid-360)| `sensor_msgs/PointCloud2`     | ⚠️ Ported (`config/mid360.yaml`)          |
| `livox`      | Livox (e.g. AVIA)      | `livox_ros_driver2/CustomMsg`      | ⚠️ Ported, untested — **optional driver** |

### Enabling Livox

> **Mid-360 / Livox as PointCloud2:** if your Livox already publishes
> `sensor_msgs/PointCloud2` (the `livox_ros_driver2` default, `xfer_format=0`),
> use `lidar_type: livox_points` (see `config/mid360.yaml`) — no extra driver is
> needed. The section below is only for the raw `CustomMsg` stream.

Unlike the other sensors, Livox does not publish `PointCloud2`; it uses the
`CustomMsg` type from the ROS 2 Livox driver. That driver is **not** a hard
dependency of this package (so `rosdep` works without it). Livox support is
compiled in only when [`livox_ros_driver2`](https://github.com/Livox-SDK/livox_ros_driver2)
is present in your workspace:

```bash
# 1. Add livox_ros_driver2 to your workspace (see its README for the SDK steps),
#    then rebuild ig_lio so it detects the driver:
colcon build --packages-select ig_lio
# Look for "Livox support ENABLED" in the build output.

# 2. Run with the bundled AVIA config:
ros2 launch ig_lio lio_livox.launch.py
```

If the driver is absent, the package still builds; selecting `lidar_type: livox`
then exits with a message telling you to install the driver and rebuild. The
Livox preprocessing is ported verbatim from upstream and has **not** been
validated in this ROS 2 port — tune `config/avia.yaml` for your unit.

---

## Configuration

All parameters live in `config/ouster128.yaml`. Copy and adapt it for other
LiDAR/IMU combos. The most relevant ones:

| Parameter          | Default         | Purpose                                                                 |
|--------------------|-----------------|-------------------------------------------------------------------------|
| `lidar_topic`      | —               | LiDAR point cloud topic to subscribe to                                 |
| `imu_topic`        | —               | IMU topic to subscribe to                                               |
| `lidar_type`       | `ouster`        | `ouster`, `velodyne`, `M1600`, `Hesai`, `livox_points`, or `livox` (case-sensitive) |
| `qos_reliability`  | `best_effort`   | `best_effort` matches any publisher; `reliable` only matches reliable   |
| `odom_frame`       | `odom`          | global/world frame (odometry, scans and path live here)                 |
| `base_frame`       | `base_link`     | moving body frame the estimated pose refers to                          |
| `t_imu_lidar`      | —               | translation mapping points from the **LiDAR** frame to the **IMU** frame|
| `R_imu_lidar`      | —               | rotation (row-major 3×3) for the same LiDAR → IMU mapping               |
| `result_directory` | `""`            | output dir for `lio_odom.txt`; empty -> the package's own `result/` dir  |

⚠️ The extrinsics (`t_imu_lidar` / `R_imu_lidar`) must map points **from the
LiDAR frame to the IMU frame**, not the other way around.

### QoS

If a bag/driver uses a different QoS than the node, the subscription may not
connect. Check a topic's reliability with:

```bash
ros2 topic info -v <topic>   # look at the "Reliability" line
```

The default `qos_reliability: best_effort` connects to both best-effort and
reliable publishers, so it works with most bags out of the box. Set it to
`reliable` only if you need guaranteed delivery and your source offers it.

---

## Trajectory output

The estimated trajectory is written to `lio_odom.txt` (location set by
`result_directory`) in **TUM format** — one pose per line, 8 space-separated
columns, directly loadable by [`evo`](https://github.com/MichaelGrupp/evo)
(`evo_traj tum lio_odom.txt`):

```
timestamp  tx ty tz  qx qy qz qw
```

| Column | Field       | Meaning                                   |
|--------|-------------|-------------------------------------------|
| 1      | `timestamp` | LiDAR scan end time, in seconds           |
| 2–4    | `tx ty tz`  | position (m) in the `odom_frame`          |
| 5–8    | `qx qy qz qw` | orientation quaternion (x, y, z, w)     |
