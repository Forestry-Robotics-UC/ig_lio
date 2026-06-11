#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "ig_lio/pointcloud_preprocess.h"
#include "ig_lio/timer.h"

extern Timer timer;

void PointCloudPreprocess::Process(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
    pcl::PointCloud<PointType>::Ptr& cloud_out) {
  switch (config_.lidar_type) {
  case LidarType::VELODYNE:
    ProcessVelodyne(msg, cloud_out);
    break;
  case LidarType::OUSTER:
    ProcessOuster(msg, cloud_out);
    break;
  case LidarType::HESAI:
    ProcessHesai(msg, cloud_out);
    break;
  case LidarType::VELODYNEM1600:
    ProcessVelodyneM1600(msg, cloud_out);
    break;
  default:
    LOG(INFO) << "Error LiDAR Type!!!" << std::endl;
    exit(0);
  }
}

void PointCloudPreprocess::ProcessVelodyne(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
    pcl::PointCloud<PointType>::Ptr& cloud_out) {
  pcl::PointCloud<VelodynePointXYZIRT> cloud_origin;
  pcl::fromROSMsg(*msg, cloud_origin);

  double omega_l = 3.61;
  std::vector<bool> is_first(num_scans_, true);
  std::vector<double> yaw_fp(num_scans_, 0.0);
  std::vector<float> yaw_last(num_scans_, 0.0);
  std::vector<float> time_last(num_scans_, 0.0);
  if (cloud_origin.back().time > 0) {
    has_time_ = true;
  } else {
    LOG(INFO) << "origin cloud has not timestamp";
    has_time_ = false;
  }

  cloud_out->reserve(cloud_origin.size());

  for (size_t i = 0; i < cloud_origin.size(); ++i) {
    if ((i % config_.point_filter_num == 0) && !HasInf(cloud_origin.at(i)) &&
        !HasNan(cloud_origin.at(i))) {
      PointType point;
      point.normal_x = 0;
      point.normal_y = 0;
      point.normal_z = 0;
      point.x = cloud_origin.at(i).x;
      point.y = cloud_origin.at(i).y;
      point.z = cloud_origin.at(i).z;
      point.intensity = cloud_origin.at(i).intensity;
      if (has_time_) {
        point.curvature = cloud_origin.at(i).time * config_.time_scale;
      } else {
        int layer = cloud_origin.points[i].ring;
        double yaw_angle = atan2(point.y, point.x) * 57.2957;

        if (is_first[layer]) {
          yaw_fp[layer] = yaw_angle;
          is_first[layer] = false;
          point.curvature = 0.0;
          yaw_last[layer] = yaw_angle;
          time_last[layer] = point.curvature;
          continue;
        }

        if (yaw_angle <= yaw_fp[layer]) {
          point.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
        } else {
          point.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
        }

        if (point.curvature < time_last[layer])
          point.curvature += 360.0 / omega_l;

        if (!std::isfinite(point.curvature)) {
          continue;
        }

        yaw_last[layer] = yaw_angle;
        time_last[layer] = point.curvature;
      }

      cloud_out->push_back(point);
    }
  }
}

void PointCloudPreprocess::ProcessVelodyneM1600(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
    pcl::PointCloud<PointType>::Ptr& cloud_out) {
  pcl::PointCloud<VelodyneM1600PointXYZIRT> cloud_origin;
  pcl::fromROSMsg(*msg, cloud_origin);

  cloud_out->clear();
  if (cloud_origin.empty())
    return;

  cloud_out->reserve(cloud_origin.size());

  double time_head_sec = cloud_origin.points[0].timestampSec;
  double time_head_nsec = cloud_origin.points[0].timestampNsec;
  double time_head = time_head_sec + time_head_nsec * 1e-9;

  for (size_t i = 0; i < cloud_origin.size(); ++i) {
    if ((i % config_.point_filter_num == 0) && !HasInf(cloud_origin.at(i)) &&
        !HasNan(cloud_origin.at(i))) {
      PointType point;
      point.normal_x = 0;
      point.normal_y = 0;
      point.normal_z = 0;
      point.x = cloud_origin.at(i).x;
      point.y = cloud_origin.at(i).y;
      point.z = cloud_origin.at(i).z;
      point.intensity = cloud_origin.at(i).intensity;

      double current_sec = cloud_origin.at(i).timestampSec;
      double current_nsec = cloud_origin.at(i).timestampNsec;
      double current_time = current_sec + current_nsec * 1e-9;

      point.curvature = (current_time - time_head) * 1000.0;
      cloud_out->push_back(point);
    }
  }
}

void PointCloudPreprocess::ProcessHesai(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
    pcl::PointCloud<PointType>::Ptr& cloud_out) {
  pcl::PointCloud<HesaiPointXYZIRT> cloud_origin;
  pcl::fromROSMsg(*msg, cloud_origin);
  if (cloud_origin.empty()) return;
  double time_begin = cloud_origin.points[0].timestamp;
  for (size_t i = 0; i < cloud_origin.size(); ++i) {
    if ((i % config_.point_filter_num == 0) && !HasInf(cloud_origin.at(i)) &&
        !HasNan(cloud_origin.at(i))) {
      PointType point;
      point.normal_x = 0;
      point.normal_y = 0;
      point.normal_z = 0;
      point.x = cloud_origin.at(i).x;
      point.y = cloud_origin.at(i).y;
      point.z = cloud_origin.at(i).z;
      point.intensity = cloud_origin.at(i).intensity;
      double current_time = cloud_origin.at(i).timestamp;
      point.curvature = (current_time - time_begin) * 1000.0;
      cloud_out->push_back(point);
    }
  }
}

#ifdef HAVE_LIVOX
void PointCloudPreprocess::ProcessLivox(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg,
    pcl::PointCloud<PointType>::Ptr& cloud_out) {
  cloud_out->clear();
  cloud_out->reserve(msg->point_num);

  for (size_t i = 1; i < msg->point_num; ++i) {
    // Keep only valid scan lines and good returns: tag bits 0x30 select the
    // spatial/intensity confidence; 0x10 (high) or 0x00 (normal) are kept.
    if ((msg->points[i].line < num_scans_) &&
        (((msg->points[i].tag & 0x30) == 0x10) ||
         ((msg->points[i].tag & 0x30) == 0x00)) &&
        (i % config_.point_filter_num == 0) && !HasInf(msg->points[i]) &&
        !HasNan(msg->points[i]) && !IsNear(msg->points[i], msg->points[i - 1])) {
      PointType point;
      point.normal_x = 0;
      point.normal_y = 0;
      point.normal_z = 0;
      point.x = msg->points[i].x;
      point.y = msg->points[i].y;
      point.z = msg->points[i].z;
      point.intensity = msg->points[i].reflectivity;
      // offset_time is nanoseconds from the scan start; curvature is in ms,
      // matching the per-cloud relative-time convention used for the others.
      point.curvature = msg->points[i].offset_time * 1e-6;
      cloud_out->push_back(point);
    }
  }
}
#endif

void PointCloudPreprocess::ProcessOuster(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
    pcl::PointCloud<PointType>::Ptr& cloud_out) {
  pcl::PointCloud<OusterPointXYZIRT> cloud_origin;
  pcl::fromROSMsg(*msg, cloud_origin);

  if (cloud_origin.empty()) return;
  const uint32_t t_start = cloud_origin.points[0].t;

  for (size_t i = 0; i < cloud_origin.size(); ++i) {
    if ((i % config_.point_filter_num == 0) && !HasInf(cloud_origin.at(i)) &&
        !HasNan(cloud_origin.at(i))) {
      PointType point;
      point.normal_x = 0;
      point.normal_y = 0;
      point.normal_z = 0;
      point.x = cloud_origin.at(i).x;
      point.y = cloud_origin.at(i).y;
      point.z = cloud_origin.at(i).z;
      point.intensity = cloud_origin.at(i).intensity;
      point.curvature =
          (cloud_origin.at(i).t - t_start) * config_.time_scale * 1000.0;
      cloud_out->push_back(point);
    }
  }
}

template <typename T>
inline bool PointCloudPreprocess::HasInf(const T& p) {
  return (std::isinf(p.x) || std::isinf(p.y) || std::isinf(p.z));
}

template <typename T>
inline bool PointCloudPreprocess::HasNan(const T& p) {
  return (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z));
}

template <typename T>
inline bool PointCloudPreprocess::IsNear(const T& p1, const T& p2) {
  return ((abs(p1.x - p2.x) < 1e-7) || (abs(p1.y - p2.y) < 1e-7) ||
          (abs(p1.z - p2.z) < 1e-7));
}