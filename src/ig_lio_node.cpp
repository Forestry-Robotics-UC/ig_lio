#include <csignal>
#include <fstream>
#include <mutex>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#ifdef HAVE_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <boost/filesystem.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "ig_lio/lio.h"
#include "ig_lio/logger.hpp"
#include "ig_lio/pointcloud_preprocess.h"
#include "ig_lio/timer.h"

namespace fs = boost::filesystem;

// ── Globals ──────────────────────────────────────────────────────────────────

LidarType lidar_type = LidarType::HESAI;
constexpr double kAccScale = 9.80665;
bool enable_acc_correct = true;
bool enable_undistort = true;
bool enable_ahrs_initalization = false;
Eigen::Matrix4d T_imu_lidar;

// Parameters used to synchronize livox time with the external IMU
double timediff_lidar_wrt_imu = 0.0;
double lidar_timestamp = 0.0;
double imu_timestamp = 0.0;
bool timediff_correct_flag = false;
std::mutex buff_mutex;

// Data deques
std::deque<std::pair<double, pcl::PointCloud<PointType>::Ptr>> cloud_buff;
std::deque<sensor_msgs::msg::Imu> imu_buff;
std::deque<nav_msgs::msg::Odometry> gnss_buff;

// ROS2 publishers
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_scan_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_scan_pub;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
nav_msgs::msg::Path path_array;

// TF broadcaster (initialised in main after node is created)
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

// Frame names (overridable via parameters; set in main before spinning).
// odom_frame  = global/world frame that odometry, scans and path live in.
// base_frame  = moving body frame the estimated pose refers to.
std::string g_odom_frame = "odom";
std::string g_base_frame = "base_link";

Timer timer;
std::shared_ptr<PointCloudPreprocess> cloud_preprocess_ptr;
SensorMeasurement sensor_measurement;
std::shared_ptr<LIO> lio_ptr;
pcl::VoxelGrid<PointType> voxel_filter;
std::fstream odom_stream;

// Node handle kept as a global so callbacks can reach it for logging
rclcpp::Node::SharedPtr g_node;

// ── IMU callback ─────────────────────────────────────────────────────────────

void ImuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg_ptr)
{
  static double last_imu_timestamp = 0.0;
  static sensor_msgs::msg::Imu last_imu = *msg_ptr;
  // EMA filter parameters
  static double a = 0.8;
  static double b = 1.0 - a;

  sensor_msgs::msg::Imu imu_msg = *msg_ptr;
  imu_timestamp = rclcpp::Time(imu_msg.header.stamp).seconds();

  if (std::abs(timediff_lidar_wrt_imu) > 0.1 && timediff_correct_flag) {
    imu_msg.header.stamp =
        rclcpp::Time(static_cast<int64_t>((imu_timestamp + timediff_lidar_wrt_imu) * 1e9));
  }

  {
    std::lock_guard<std::mutex> lock(buff_mutex);

    if (imu_timestamp < last_imu_timestamp) {
      LOG(WARNING) << "imu loop back, clear buffer";
      imu_buff.clear();
    }
    last_imu_timestamp = imu_timestamp;

    // EMA filter for accelerometer
    imu_msg.linear_acceleration.x =
        msg_ptr->linear_acceleration.x * a + last_imu.linear_acceleration.x * b;
    imu_msg.linear_acceleration.y =
        msg_ptr->linear_acceleration.y * a + last_imu.linear_acceleration.y * b;
    imu_msg.linear_acceleration.z =
        msg_ptr->linear_acceleration.z * a + last_imu.linear_acceleration.z * b;
    last_imu = *msg_ptr;

    // Some Livox datasets omit the gravitational constant in the accelerometer
    if (enable_acc_correct) {
      imu_msg.linear_acceleration.x *= kAccScale;
      imu_msg.linear_acceleration.y *= kAccScale;
      imu_msg.linear_acceleration.z *= kAccScale;
    }

    imu_buff.push_back(imu_msg);
  }
}

// ── LiDAR callback (Velodyne / Ouster / Hesai) ───────────────────────────────

void CloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  static double last_lidar_timestamp = 0.0;
  timer.Evaluate(
      [&]() {
        lidar_timestamp = rclcpp::Time(msg->header.stamp).seconds();

        CloudPtr cloud_ptr(new CloudType());
        cloud_preprocess_ptr->Process(msg, cloud_ptr);

        {
          std::lock_guard<std::mutex> lock(buff_mutex);

          if (lidar_timestamp < last_lidar_timestamp) {
            LOG(WARNING) << "lidar loop back, clear buffer";
            cloud_buff.clear();
          }
          last_lidar_timestamp = lidar_timestamp;

          cloud_buff.push_back(
              std::make_pair(rclcpp::Time(msg->header.stamp).seconds(), cloud_ptr));
        }
      },
      "Cloud Preprocess (Standard)");
}

#ifdef HAVE_LIVOX
// ── LiDAR callback (Livox CustomMsg) ─────────────────────────────────────────
// Livox does not publish PointCloud2, so it gets its own callback feeding the
// same cloud_buff. Ported from upstream; not validated on hardware here.

void LivoxCloudCallBack(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
{
  static double last_lidar_timestamp = 0.0;
  timer.Evaluate(
      [&]() {
        lidar_timestamp = rclcpp::Time(msg->header.stamp).seconds();

        CloudPtr cloud_ptr(new CloudType());
        cloud_preprocess_ptr->ProcessLivox(msg, cloud_ptr);

        {
          std::lock_guard<std::mutex> lock(buff_mutex);

          if (lidar_timestamp < last_lidar_timestamp) {
            LOG(WARNING) << "lidar loop back, clear buffer";
            cloud_buff.clear();
          }
          last_lidar_timestamp = lidar_timestamp;

          cloud_buff.push_back(
              std::make_pair(rclcpp::Time(msg->header.stamp).seconds(), cloud_ptr));
        }
      },
      "Cloud Preprocess (Livox)");
}
#endif

// ── Measurement synchronisation ──────────────────────────────────────────────

bool SyncMeasurements()
{
  static bool measurement_pushed = false;
  static bool process_lidar = false;
  static SensorMeasurement local_sensor_measurement;
  static double lidar_mean_scantime = 0.0;
  static size_t lidar_scan_num = 0;

  if (cloud_buff.empty() || imu_buff.empty()) {
    return false;
  }

  std::lock_guard<std::mutex> lock(buff_mutex);

  double lidar_end_time = 0.0;
  if (!measurement_pushed) {
    if (!process_lidar) {
      CloudPtr cloud_sort(new CloudType());
      *cloud_sort = *cloud_buff.front().second;
      std::sort(cloud_sort->points.begin(),
                cloud_sort->points.end(),
                [](const PointType& x, const PointType& y) -> bool {
                  return (x.curvature < y.curvature);
                });
      local_sensor_measurement.cloud_ptr_ = cloud_sort;
      local_sensor_measurement.bag_time_ = cloud_buff.front().first;
      if (!local_sensor_measurement.cloud_ptr_->points.empty()) {
        local_sensor_measurement.lidar_start_time_ =
            cloud_buff.front().first +
            local_sensor_measurement.cloud_ptr_->points.front().curvature /
                (double)(1000);
      } else {
        local_sensor_measurement.lidar_start_time_ = cloud_buff.front().first;
      }

      if (local_sensor_measurement.cloud_ptr_->size() <= 1) {
        LOG(WARNING) << "Too Few Points in Cloud!!!";
        lidar_end_time =
            local_sensor_measurement.lidar_start_time_ + lidar_mean_scantime;
      } else if (local_sensor_measurement.cloud_ptr_->points.back().curvature /
                     (double)(1000) <
                 0.5 * lidar_mean_scantime) {
        lidar_end_time =
            local_sensor_measurement.lidar_start_time_ + lidar_mean_scantime;
      } else {
        lidar_scan_num++;
        lidar_end_time =
            local_sensor_measurement.bag_time_ +
            local_sensor_measurement.cloud_ptr_->points.back().curvature /
                (double)(1000);
        lidar_mean_scantime +=
            ((local_sensor_measurement.cloud_ptr_->points.back().curvature -
              local_sensor_measurement.cloud_ptr_->points.front().curvature) /
                 (double)(1000) -
             lidar_mean_scantime) /
            (double)(lidar_scan_num);
      }

      local_sensor_measurement.lidar_end_time_ =
          enable_undistort ? lidar_end_time : local_sensor_measurement.bag_time_;

      process_lidar = true;
    }

    // NOTE: GNSS via nav_msgs::msg::Odometry is retained; wire up a subscriber
    // in main() if needed (gnss_buff is populated elsewhere).
    bool get_gnss_measurement = false;
    while (!gnss_buff.empty()) {
      double gnss_t = rclcpp::Time(gnss_buff.front().header.stamp).seconds();
      if (gnss_t > sensor_measurement.bag_time_) {
        if (gnss_t > local_sensor_measurement.bag_time_) {
          LOG(INFO) << "gnss too new";
          break;
        }

        if ((int)(gnss_buff.front().twist.covariance[0]) == 1) {
          sensor_measurement.gnss_status_ = GNSSStatus::RTK_FIXED;
        } else {
          sensor_measurement.gnss_status_ = GNSSStatus::NONE;
        }

        sensor_measurement.measurement_type_ = MeasurementType::GNSS;
        sensor_measurement.bag_time_ = gnss_t;
        sensor_measurement.lidar_start_time_ = gnss_t;
        sensor_measurement.lidar_end_time_ = gnss_t;

        sensor_measurement.has_gnss_ori_ = false;
        Eigen::Vector3d temp_t(gnss_buff.front().pose.pose.position.x,
                               gnss_buff.front().pose.pose.position.y,
                               gnss_buff.front().pose.pose.position.z);

        auto& ori = gnss_buff.front().pose.pose.orientation;
        double qnorm_sq = ori.w * ori.w + ori.x * ori.x +
                          ori.y * ori.y + ori.z * ori.z;
        if (qnorm_sq < 1.0) {
          sensor_measurement.gnss_pose_.block<3, 3>(0, 0) =
              Eigen::Matrix3d::Identity();
          sensor_measurement.gnss_pose_.block<3, 1>(0, 3) = temp_t;
          LOG(INFO) << "get gnss measurement.";
        } else {
          Eigen::Quaterniond temp_q(ori.w, ori.x, ori.y, ori.z);
          sensor_measurement.gnss_pose_.block<3, 3>(0, 0) =
              temp_q.toRotationMatrix();
          sensor_measurement.gnss_pose_.block<3, 1>(0, 3) = temp_t;
          sensor_measurement.has_gnss_ori_ = true;
          LOG(INFO) << "get gnss measurement with ori.";
        }

        get_gnss_measurement = true;
        break;
      } else {
        gnss_buff.pop_front();
        LOG(INFO) << "gnss too old";
      }
    }

    if (!get_gnss_measurement) {
      sensor_measurement = local_sensor_measurement;
      sensor_measurement.measurement_type_ = MeasurementType::LIDAR;
    }

    measurement_pushed = true;

    if (sensor_measurement.measurement_type_ == MeasurementType::LIDAR) {
      cloud_buff.pop_front();
      process_lidar = false;
    } else if (sensor_measurement.measurement_type_ == MeasurementType::GNSS) {
      gnss_buff.pop_front();
    }
  }

  double latest_imu_t =
      rclcpp::Time(imu_buff.back().header.stamp).seconds();
  if (latest_imu_t < sensor_measurement.lidar_end_time_) {
    return false;
  }

  sensor_measurement.imu_buff_.clear();
  while (!imu_buff.empty()) {
    double imu_time = rclcpp::Time(imu_buff.front().header.stamp).seconds();
    if (imu_time < sensor_measurement.lidar_end_time_) {
      sensor_measurement.imu_buff_.push_back(imu_buff.front());
      imu_buff.pop_front();
    } else {
      break;
    }
  }
  sensor_measurement.imu_buff_.push_back(imu_buff.front());

  measurement_pushed = false;
  return true;
}

// ── Main processing loop ─────────────────────────────────────────────────────

void Process()
{
  // Step 1: Time synchronisation
  if (!SyncMeasurements()) {
    return;
  }

  // Step 2: AHRS or static initialisation
  if (!lio_ptr->IsInit()) {
    if (enable_ahrs_initalization) {
      lio_ptr->AHRSInitialization(sensor_measurement);
    } else {
      lio_ptr->StaticInitialization(sensor_measurement);
    }
    return;
  }

  // Step 3: IMU prediction
  for (size_t i = 0; i < sensor_measurement.imu_buff_.size(); ++i) {
    double time;
    if (i == sensor_measurement.imu_buff_.size() - 1) {
      time = sensor_measurement.lidar_end_time_;
    } else {
      time = rclcpp::Time(sensor_measurement.imu_buff_.at(i).header.stamp).seconds();
    }
    Eigen::Vector3d acc(
        sensor_measurement.imu_buff_.at(i).linear_acceleration.x,
        sensor_measurement.imu_buff_.at(i).linear_acceleration.y,
        sensor_measurement.imu_buff_.at(i).linear_acceleration.z);
    Eigen::Vector3d gyr(
        sensor_measurement.imu_buff_.at(i).angular_velocity.x,
        sensor_measurement.imu_buff_.at(i).angular_velocity.y,
        sensor_measurement.imu_buff_.at(i).angular_velocity.z);
    lio_ptr->Predict(time, acc, gyr);
  }

  if (sensor_measurement.cloud_ptr_->size() <= 1) {
    LOG(WARNING) << "no point, skip this scan";
    return;
  }

  // Step 4: Measurement update
  timer.Evaluate([&] { lio_ptr->MeasurementUpdate(sensor_measurement); },
                 "measurement update");

  // Step 5: Publish to RViz
  Eigen::Matrix4d result_pose = lio_ptr->GetCurrentPose();
  Eigen::Matrix<double, 15, 15> P = lio_ptr->GetCovariance();

  rclcpp::Time stamp(
      static_cast<int64_t>(sensor_measurement.lidar_end_time_ * 1e9));

  // Odometry message
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = g_odom_frame;
  odom_msg.child_frame_id = g_base_frame;
  odom_msg.header.stamp = stamp;
  Eigen::Quaterniond temp_q(result_pose.block<3, 3>(0, 0));
  odom_msg.pose.pose.orientation.x = temp_q.x();
  odom_msg.pose.pose.orientation.y = temp_q.y();
  odom_msg.pose.pose.orientation.z = temp_q.z();
  odom_msg.pose.pose.orientation.w = temp_q.w();
  odom_msg.pose.pose.position.x = result_pose(0, 3);
  odom_msg.pose.pose.position.y = result_pose(1, 3);
  odom_msg.pose.pose.position.z = result_pose(2, 3);

  // Covariance: ig-lio P_ order is Ori(0-2), Pos(3-5), Vel(6-8), Ba(9-11), Bg(12-14)
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      odom_msg.pose.covariance[(i) * 6 + (j)]     = P(3 + i, 3 + j);  // pos-pos
      odom_msg.pose.covariance[(i) * 6 + (j + 3)] = P(3 + i, j);      // pos-ori
      odom_msg.pose.covariance[(i + 3) * 6 + (j)] = P(i, 3 + j);      // ori-pos
      odom_msg.pose.covariance[(i + 3) * 6 + (j + 3)] = P(i, j);      // ori-ori
      odom_msg.twist.covariance[(i) * 6 + (j)]    = P(6 + i, 6 + j);  // vel-vel
    }
  }
  odom_pub->publish(odom_msg);

  // TF broadcast
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = g_odom_frame;
  tf_msg.child_frame_id = g_base_frame;
  tf_msg.transform.translation.x = result_pose(0, 3);
  tf_msg.transform.translation.y = result_pose(1, 3);
  tf_msg.transform.translation.z = result_pose(2, 3);
  tf_msg.transform.rotation.x = temp_q.x();
  tf_msg.transform.rotation.y = temp_q.y();
  tf_msg.transform.rotation.z = temp_q.z();
  tf_msg.transform.rotation.w = temp_q.w();
  tf_broadcaster->sendTransform(tf_msg);

  // Publish dense scan (transformed to odom frame)
  CloudPtr trans_cloud(new CloudType());
  pcl::transformPointCloud(
      *sensor_measurement.cloud_ptr_, *trans_cloud, result_pose);
  sensor_msgs::msg::PointCloud2 scan_msg;
  pcl::toROSMsg(*trans_cloud, scan_msg);
  scan_msg.header.frame_id = g_odom_frame;
  scan_msg.header.stamp = stamp;
  current_scan_pub->publish(scan_msg);

  // Publish keyframe path and downsampled scan
  static bool is_first_keyframe = true;
  static Eigen::Matrix4d last_keyframe = result_pose;
  Eigen::Matrix4d delta_p = last_keyframe.inverse() * result_pose;
  if (is_first_keyframe ||
      delta_p.block<3, 1>(0, 3).norm() > 1.0 ||
      Sophus::SO3d(delta_p.block<3, 3>(0, 0)).log().norm() > 0.18)
  {
    if (is_first_keyframe) is_first_keyframe = false;
    last_keyframe = result_pose;

    CloudPtr cloud_DS(new CloudType());
    voxel_filter.setInputCloud(sensor_measurement.cloud_ptr_);
    voxel_filter.filter(*cloud_DS);
    CloudPtr trans_cloud_DS(new CloudType());
    pcl::transformPointCloud(*cloud_DS, *trans_cloud_DS, result_pose);
    sensor_msgs::msg::PointCloud2 keyframe_scan_msg;
    pcl::toROSMsg(*trans_cloud_DS, keyframe_scan_msg);
    keyframe_scan_msg.header.frame_id = g_odom_frame;
    keyframe_scan_msg.header.stamp = stamp;
    keyframe_scan_pub->publish(keyframe_scan_msg);

    path_array.header.stamp = stamp;
    path_array.header.frame_id = g_odom_frame;
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = stamp;
    pose_stamped.header.frame_id = g_odom_frame;
    pose_stamped.pose.position.x = result_pose(0, 3);
    pose_stamped.pose.position.y = result_pose(1, 3);
    pose_stamped.pose.position.z = result_pose(2, 3);
    pose_stamped.pose.orientation.w = temp_q.w();
    pose_stamped.pose.orientation.x = temp_q.x();
    pose_stamped.pose.orientation.y = temp_q.y();
    pose_stamped.pose.orientation.z = temp_q.z();
    path_array.poses.push_back(pose_stamped);
    path_pub->publish(path_array);
  }

  // Step 6: Save trajectory for evo evaluation
  static size_t delay_count = 0;
  if (delay_count > 50) {
    Eigen::Matrix4d lio_pose = result_pose;
    Eigen::Quaterniond lio_q(lio_pose.block<3, 3>(0, 0));
    odom_stream << std::fixed << std::setprecision(6)
                << sensor_measurement.lidar_end_time_ << " "
                << std::setprecision(15)
                << lio_pose(0, 3) << " " << lio_pose(1, 3) << " " << lio_pose(2, 3)
                << " " << lio_q.x() << " " << lio_q.y() << " " << lio_q.z()
                << " " << lio_q.w() << std::endl;
  } else {
    delay_count++;
  }
}

// ── Entry point ───────────────────────────────────────────────────────────────

bool FLAG_EXIT = false;
void SigHandle(int sig)
{
  FLAG_EXIT = true;
  RCLCPP_WARN(rclcpp::get_logger("ig_lio_node"), "catch sig %d", sig);
}

int main(int argc, char** argv)
{
  // Strip ROS2-specific args (--ros-args, --params-file, -r, etc.) BEFORE
  // initialising glog via Logger, otherwise glog's flag parser crashes on
  // unknown flags injected by the ROS2 launch system.
  std::vector<std::string> non_ros_args =
      rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char*> clean_argv;
  clean_argv.reserve(non_ros_args.size());
  for (auto& s : non_ros_args) clean_argv.push_back(const_cast<char*>(s.c_str()));
  int clean_argc = static_cast<int>(clean_argv.size());

  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("ig_lio_node");

  // Write glog output to the package's own log/ dir (source tree, baked in by
  // CMake) rather than the install/share copy, which gets overwritten on every
  // rebuild. Matches where the trajectory result/ output lands.
  Logger logger(clean_argc, clean_argv.data(), IG_LIO_SOURCE_DIR);

  // ── QoS reliability (matches whatever the bag/driver offers) ─────────────────
  // best_effort (default) subscribers match BOTH best_effort and reliable
  // publishers, so this works across bags without recompiling. Set to
  // "reliable" only when you specifically want guaranteed delivery from a
  // reliable publisher. A reliable subscriber will NOT connect to a
  // best_effort publisher.
  std::string qos_reliability;
  g_node->declare_parameter<std::string>("qos_reliability", "best_effort");
  g_node->get_parameter("qos_reliability", qos_reliability);
  const bool use_reliable = (qos_reliability == "reliable");
  if (!use_reliable && qos_reliability != "best_effort") {
    LOG(WARNING) << "Unknown qos_reliability '" << qos_reliability
                 << "', falling back to best_effort";
  }
  auto apply_reliability = [use_reliable](rclcpp::QoS qos) {
    return use_reliable ? qos.reliable() : qos.best_effort();
  };
  LOG(INFO) << "QoS reliability: "
            << (use_reliable ? "reliable" : "best_effort");

  // ── Frame names ─────────────────────────────────────────────────────────────
  // Make parent/child frames configurable so the node can coexist with bags or
  // other nodes that already publish odom->base_link (just point this at a
  // different child, e.g. base_frame:="lio_base_link").
  g_node->declare_parameter<std::string>("odom_frame", g_odom_frame);
  g_node->declare_parameter<std::string>("base_frame", g_base_frame);
  g_node->get_parameter("odom_frame", g_odom_frame);
  g_node->get_parameter("base_frame", g_base_frame);
  LOG(INFO) << "TF frames: " << g_odom_frame << " -> " << g_base_frame;

  // ── Subscribe: IMU ──────────────────────────────────────────────────────────
  std::string imu_topic;
  g_node->declare_parameter<std::string>("imu_topic", "/imu/data");
  g_node->get_parameter("imu_topic", imu_topic);
  // IMU: deep queue so a stalled loop buffers instead of dropping
  auto imu_qos = apply_reliability(rclcpp::QoS(rclcpp::KeepLast(2000)));
  auto imu_sub = g_node->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, imu_qos, ImuCallBack);

  // ── Subscribe: LiDAR ────────────────────────────────────────────────────────
  std::string lidar_topic, lidar_type_string;
  g_node->declare_parameter<std::string>("lidar_topic", "velodyne_points");
  g_node->declare_parameter<std::string>("lidar_type", "velodyne");
  g_node->get_parameter("lidar_topic", lidar_topic);
  g_node->get_parameter("lidar_type", lidar_type_string);

  if (lidar_type_string == "velodyne") {
    lidar_type = LidarType::VELODYNE;
  } else if (lidar_type_string == "M1600") {
    lidar_type = LidarType::VELODYNEM1600;
  } else if (lidar_type_string == "Hesai") {
    lidar_type = LidarType::HESAI;
  } else if (lidar_type_string == "ouster") {
    lidar_type = LidarType::OUSTER;
  } else if (lidar_type_string == "livox") {
#ifdef HAVE_LIVOX
    lidar_type = LidarType::LIVOX;
#else
    LOG(ERROR) << "lidar_type 'livox' selected but ig_lio was built without "
                  "livox_ros_driver2. Install the driver in your workspace and "
                  "rebuild to enable Livox support.";
    return 1;
#endif
  } else {
    LOG(ERROR) << "Error lidar type!";
    return 1;
  }

  // LiDAR: depth 10; clouds are big & low-rate
  auto lidar_qos = apply_reliability(rclcpp::QoS(rclcpp::KeepLast(10)));
  // Livox uses its own CustomMsg subscription; everything else is PointCloud2.
  rclcpp::SubscriptionBase::SharedPtr cloud_sub;
#ifdef HAVE_LIVOX
  if (lidar_type == LidarType::LIVOX) {
    cloud_sub = g_node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lidar_topic, lidar_qos, LivoxCloudCallBack);
  } else
#endif
  {
    cloud_sub = g_node->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic, lidar_qos, CloudCallBack);
  }

  // ── Parameters: point-cloud pre-processing ──────────────────────────────────
  double time_scale;
  int point_filter_num;
  g_node->declare_parameter<double>("time_scale", 1.0);
  g_node->declare_parameter<int>("point_filter_num", 1);
  g_node->get_parameter("time_scale", time_scale);
  g_node->get_parameter("point_filter_num", point_filter_num);

  LOG(INFO) << "\ntime_scale: " << time_scale << "\npoint_filter_num: " << point_filter_num;

  PointCloudPreprocess::Config cloud_preprocess_config;
  cloud_preprocess_config.lidar_type = lidar_type;
  cloud_preprocess_config.point_filter_num = point_filter_num;
  cloud_preprocess_config.time_scale = time_scale;
  cloud_preprocess_ptr =
      std::make_shared<PointCloudPreprocess>(cloud_preprocess_config);

  // ── Parameters: LIO ─────────────────────────────────────────────────────────
  double scan_resolution, voxel_map_resolution;
  int max_iterations;
  g_node->declare_parameter<double>("scan_resolution", 1.0);
  g_node->declare_parameter<double>("voxel_map_resolution", 1.0);
  g_node->declare_parameter<int>("max_iterations", 30);
  g_node->get_parameter("scan_resolution", scan_resolution);
  g_node->get_parameter("voxel_map_resolution", voxel_map_resolution);
  g_node->get_parameter("max_iterations", max_iterations);

  double acc_cov, gyr_cov, bg_cov, ba_cov;
  double init_ori_cov, init_pos_cov, init_vel_cov, init_ba_cov, init_bg_cov;
  double gravity;
  g_node->declare_parameter<double>("acc_cov", 1.0);
  g_node->declare_parameter<double>("gyr_cov", 1.0);
  g_node->declare_parameter<double>("ba_cov", 1.0);
  g_node->declare_parameter<double>("bg_cov", 1.0);
  g_node->declare_parameter<double>("init_ori_cov", 1.0);
  g_node->declare_parameter<double>("init_pos_cov", 1.0);
  g_node->declare_parameter<double>("init_vel_cov", 1.0);
  g_node->declare_parameter<double>("init_ba_cov", 1.0);
  g_node->declare_parameter<double>("init_bg_cov", 1.0);
  g_node->declare_parameter<double>("gravity", 9.80665);
  g_node->get_parameter("acc_cov", acc_cov);
  g_node->get_parameter("gyr_cov", gyr_cov);
  g_node->get_parameter("ba_cov", ba_cov);
  g_node->get_parameter("bg_cov", bg_cov);
  g_node->get_parameter("init_ori_cov", init_ori_cov);
  g_node->get_parameter("init_pos_cov", init_pos_cov);
  g_node->get_parameter("init_vel_cov", init_vel_cov);
  g_node->get_parameter("init_ba_cov", init_ba_cov);
  g_node->get_parameter("init_bg_cov", init_bg_cov);
  g_node->get_parameter("gravity", gravity);

  double gicp_constraints_gain, point2plane_constraints_gain;
  bool enable_outlier_rejection;
  g_node->declare_parameter<double>("gicp_constraints_gain", 1.0);
  g_node->declare_parameter<double>("point2plane_constraints_gain", 1.0);
  g_node->declare_parameter<bool>("enable_undistort", true);
  g_node->declare_parameter<bool>("enable_outlier_rejection", false);
  g_node->declare_parameter<bool>("enable_acc_correct", false);
  g_node->declare_parameter<bool>("enable_ahrs_initalization", false);
  g_node->get_parameter("gicp_constraints_gain", gicp_constraints_gain);
  g_node->get_parameter("point2plane_constraints_gain", point2plane_constraints_gain);
  g_node->get_parameter("enable_undistort", enable_undistort);
  g_node->get_parameter("enable_outlier_rejection", enable_outlier_rejection);
  g_node->get_parameter("enable_acc_correct", enable_acc_correct);
  g_node->get_parameter("enable_ahrs_initalization", enable_ahrs_initalization);

  double min_radius, max_radius;
  g_node->declare_parameter<double>("min_radius", 1.0);
  g_node->declare_parameter<double>("max_radius", 1.0);
  g_node->get_parameter("min_radius", min_radius);
  g_node->get_parameter("max_radius", max_radius);

  LOG(INFO) << "\nscan_resolution: "             << scan_resolution
            << "\nvoxel_map_resolution: "       << voxel_map_resolution
            << "\nmax_iterations: "             << max_iterations
            << "\nacc_cov: "                    << acc_cov
            << "\ngyr_cov: "                    << gyr_cov
            << "\nba_cov: "                     << ba_cov
            << "\nbg_cov: "                     << bg_cov
            << "\ngravity: "                    << gravity
            << "\ninit_ori_cov: "               << init_ori_cov
            << "\ninit_pos_cov: "               << init_pos_cov
            << "\ninit_vel_cov: "               << init_vel_cov
            << "\ninit_ba_cov: "                << init_ba_cov
            << "\ninit_bg_cov: "                << init_bg_cov
            << "\ngicp_constraints_gain: "      << gicp_constraints_gain
            << "\npoint2plane_constraints_gain: "<< point2plane_constraints_gain
            << "\nenable_undistort: "           << enable_undistort
            << "\nenable_acc_correct: "         << enable_acc_correct
            << "\nenable_outlier_rejection: "   << enable_outlier_rejection
            << "\nenable_ahrs_initalization: "  << enable_ahrs_initalization
            << "\nmin_radius: "                 << min_radius
            << "\nmax_radius: "                 << max_radius;

  // ── Extrinsics ──────────────────────────────────────────────────────────────
  T_imu_lidar = Eigen::Matrix4d::Identity();
  std::vector<double> t_imu_lidar_v, R_imu_lidar_v;
  g_node->declare_parameter<std::vector<double>>("t_imu_lidar", std::vector<double>());
  g_node->declare_parameter<std::vector<double>>("R_imu_lidar", std::vector<double>());
  g_node->get_parameter("t_imu_lidar", t_imu_lidar_v);
  g_node->get_parameter("R_imu_lidar", R_imu_lidar_v);

  T_imu_lidar.block<3, 1>(0, 3) =
      Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
          t_imu_lidar_v.data(), 3, 1);
  T_imu_lidar.block<3, 3>(0, 0) =
      Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
          R_imu_lidar_v.data(), 3, 3);
  LOG(INFO) << "Extrinsic:\n" << T_imu_lidar;

  // ── Build LIO ───────────────────────────────────────────────────────────────
  LIO::Config lio_config;
  lio_config.acc_cov = acc_cov;
  lio_config.gyr_cov = gyr_cov;
  lio_config.ba_cov = ba_cov;
  lio_config.bg_cov = bg_cov;
  lio_config.gravity = gravity;
  lio_config.init_ori_cov = init_ori_cov;
  lio_config.init_pos_cov = init_pos_cov;
  lio_config.init_vel_cov = init_vel_cov;
  lio_config.init_ba_cov = init_ba_cov;
  lio_config.init_bg_cov = init_bg_cov;
  lio_config.gicp_constraint_gain = gicp_constraints_gain;
  lio_config.point2plane_constraint_gain = point2plane_constraints_gain;
  lio_config.enable_outlier_rejection = enable_outlier_rejection;
  lio_config.enable_undistort = enable_undistort;
  lio_config.max_iterations = max_iterations;
  lio_config.current_scan_resolution = scan_resolution;
  lio_config.voxel_map_resolution = voxel_map_resolution;
  lio_config.min_radius = min_radius;
  lio_config.max_radius = max_radius;
  lio_config.T_imu_lidar = T_imu_lidar;
  lio_ptr = std::make_shared<LIO>(lio_config);

  // ── Publishers ──────────────────────────────────────────────────────────────
  odom_pub = g_node->create_publisher<nav_msgs::msg::Odometry>("/lio_odom", 10000);
  current_scan_pub =
      g_node->create_publisher<sensor_msgs::msg::PointCloud2>("current_scan", 10000);
  keyframe_scan_pub =
      g_node->create_publisher<sensor_msgs::msg::PointCloud2>("keyframe_scan", 10000);
  path_pub = g_node->create_publisher<nav_msgs::msg::Path>("/path", rclcpp::QoS(10000).transient_local());

  tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(g_node);

  voxel_filter.setLeafSize(0.5f, 0.5f, 0.5f);

  // ── Trajectory output file ───────────────────────────────────────────────────
  // result_directory lets the user pick where lio_odom.txt lands. Empty -> the
  // package's own result/ dir (IG_LIO_SOURCE_DIR is baked in at build time by
  // CMake, so this persists across rebuilds).
  std::string result_directory;
  g_node->declare_parameter<std::string>("result_directory", "");
  g_node->get_parameter("result_directory", result_directory);
  fs::path result_dir = result_directory.empty()
                            ? fs::path(IG_LIO_SOURCE_DIR) / "result"
                            : fs::path(result_directory);
  fs::path result_path = result_dir / "lio_odom.txt";
  if (!fs::exists(result_path.parent_path())) {
    fs::create_directories(result_path.parent_path());
  }
  LOG(INFO) << "Trajectory output: " << result_path;
  odom_stream.open(result_path, std::ios::out);
  if (!odom_stream.is_open()) {
    LOG(ERROR) << "failed to open: " << result_path;
    return 1;
  }

  // ── Main loop ────────────────────────────────────────────────────────────────
  signal(SIGINT, SigHandle);
  rclcpp::Rate rate(5000);
  while (rclcpp::ok() && !FLAG_EXIT) {
    rclcpp::spin_some(g_node);
    Process();
    rate.sleep();
  }

  timer.PrintAll();
  rclcpp::shutdown();
  return 0;
}