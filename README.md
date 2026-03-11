# FRUC iG-LIO

Preserves the functionality of the original code, but introduces some important modifications:

- Removes dependencies (and support) of Livox LiDARs;
- Adds support to the legacy Velodyne Velarray M1600 LiDAR (by Pedro Tomás);
- Fixes the point cloud undistorting mechanism (at least with the Ouster) by fixing Ouster-related code to update old message field names, types and other small details;
- Adds a well-tuned configuration file to use with the Ouster OS1 Rev7 128 channel LiDAR + internal IMU;


### Acknowledgement:

The original repository can be found [here](https://github.com/zijiechenrobotics/ig_lio).

---

## To Install Dependencies

```bash
cd ~/catkin_ws && rosdep install --from-paths src --ignore-src -r -y
```

❗ **Note**: Change `~/catkin_ws` to your actual workspace path before running the command.

## Instructions to Use

Let's consider, as an example, that we are using the Ouster OS1 Rev7 128 channel LiDAR and internal IMU. Then, if we want to run iG-LIO online, we simple run 
`roslaunch ig_lio lio_ouster_128_online.launch` with the LiDAR driver running in parallel and iG-LIO automatically starts publishing the `map -> base_link` transform on the topic `/tf`.

If, instead we want to run it offline, we run `roslaunch ig_lio lio_ouster_128.launch` (or simple turn the `use_sim_time` parameter to true).

To use with different LiDAR/IMU combos, create a new config file or modify an existing one to adapt it to your needs, keeping in mind that the translation vector and rotation matrix provided must map the points from the LiDAR frame to the IMU frame, not the other way around!