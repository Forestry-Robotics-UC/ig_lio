#include <algorithm>

#include "ig_lio/pointcloud_preprocess.h"
#include "ig_lio/timer.h"

extern Timer timer;

void PointCloudPreprocess::Process(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
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
  case LidarType::LIVOX_POINTS:
    ProcessLivoxPointCloud2(msg, cloud_out);
    break;
  default:
    LOG(INFO) << "Error LiDAR Type!!!" << std::endl;
    exit(0);
  }
}
void PointCloudPreprocess::ProcessVelodyne(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    pcl::PointCloud<PointType>::Ptr& cloud_out) {
  pcl::PointCloud<VelodynePointXYZIRT> cloud_origin;
  pcl::fromROSMsg(*msg, cloud_origin);

  // These variables only works when no point timestamps given
  int plsize = cloud_origin.size();
  double omega_l = 3.61;  // scan angular velocity
  std::vector<bool> is_first(num_scans_, true);
  std::vector<double> yaw_fp(num_scans_, 0.0);    // yaw of first scan point
  std::vector<float> yaw_last(num_scans_, 0.0);   // yaw of last scan point
  std::vector<float> time_last(num_scans_, 0.0);  // last offset time
  if (cloud_origin.back().time > 0) {
    has_time_ = true;
  } else {
    LOG(INFO) << "origin cloud has not timestamp";
    has_time_ = false;
    double yaw_first =
        atan2(cloud_origin.points[0].y, cloud_origin.points[0].x) * 57.29578;
    double yaw_end = yaw_first;
    int layer_first = cloud_origin.points[0].ring;
    for (uint i = plsize - 1; i > 0; i--) {
      if (cloud_origin.points[i].ring == layer_first) {
        yaw_end = atan2(cloud_origin.points[i].y, cloud_origin.points[i].x) *
                  57.29578;
        break;
      }
    }
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
        // curvature unit: ms
        point.curvature = cloud_origin.at(i).time * config_.time_scale;
        // std::cout<<point.curvature<<std::endl;
        // if(point.curvature < 0){
        //     std::cout<<"time < 0 : "<<point.curvature<<std::endl;
        // }
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

        // compute offset time
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
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    pcl::PointCloud<PointType>::Ptr& cloud_out) {
  pcl::PointCloud<VelodyneM1600PointXYZIRT> cloud_origin;
  pcl::fromROSMsg(*msg, cloud_origin);

  // Clear the output cloud
  cloud_out->clear();

  if (cloud_origin.empty())
    return;

  // Reserve memory to prevent reallocation
  cloud_out->reserve(cloud_origin.size());

  // Calculate timestamp of the first point
  double time_head_sec = cloud_origin.points[0].timestampSec;
  double time_head_nsec = cloud_origin.points[0].timestampNsec;
  double time_head = time_head_sec + time_head_nsec * 1e-9;

  // Iterate through the original cloud
  for (size_t i = 0; i < cloud_origin.size(); ++i) {
    if ((i % config_.point_filter_num == 0) && !HasInf(cloud_origin.at(i)) &&
        !HasNan(cloud_origin.at(i))) {
      PointType point;
      point.normal_x = 0;
      point.normal_y = 0;
      point.normal_z = 0;
      point.x = cloud_origin.at(i).x;
      point.y = cloud_origin.at(i).y ;
      point.z = cloud_origin.at(i).z ;
      point.intensity = cloud_origin.at(i).intensity;

      // Calculate timestamp of the current point
      double current_sec = cloud_origin.at(i).timestampSec;
      double current_nsec = cloud_origin.at(i).timestampNsec;
      double current_time = current_sec + current_nsec * 1e-9;

      // Set curvature based on timestamp difference
      point.curvature = (current_time - time_head) * 1000; // Curvature unit: ms

      // Add the point to the output cloud
      cloud_out->push_back(point);
    }
  }
}
void PointCloudPreprocess::ProcessHesai(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    pcl::PointCloud<PointType>::Ptr& cloud_out) {
    pcl::PointCloud<HesaiPointXYZIRT> cloud_origin;
    pcl::fromROSMsg(*msg, cloud_origin);
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
      point.curvature = (current_time- time_begin) * 1000;
      
      cloud_out->push_back(point);
    }
  }
}




#ifdef HAVE_LIVOX
void PointCloudPreprocess::ProcessLivox(
    const livox_ros_driver2::CustomMsg::ConstPtr& msg,
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

void PointCloudPreprocess::ProcessLivoxPointCloud2(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    pcl::PointCloud<PointType>::Ptr& cloud_out) {
  pcl::PointCloud<LivoxPointXYZITLT> cloud_origin;
  pcl::fromROSMsg(*msg, cloud_origin);

  cloud_out->clear();
  if (cloud_origin.empty())
    return;
  cloud_out->reserve(cloud_origin.size());

  // timestamp is the absolute time of each point in nanoseconds. Make it
  // relative to the scan start and convert to milliseconds, matching the
  // per-cloud relative-time (curvature) convention used by the other paths.
  const double time_begin = cloud_origin.points.front().timestamp;

  for (size_t i = 1; i < cloud_origin.size(); ++i) {
    const auto& pi = cloud_origin.points[i];
    // Keep only valid scan lines and good returns: tag bits 0x30 select the
    // spatial/intensity confidence; 0x10 (high) or 0x00 (normal) are kept.
    // Mirrors the Livox CustomMsg path.
    if ((pi.line < num_scans_) &&
        (((pi.tag & 0x30) == 0x10) || ((pi.tag & 0x30) == 0x00)) &&
        (i % config_.point_filter_num == 0) && !HasInf(pi) && !HasNan(pi) &&
        !IsNear(pi, cloud_origin.points[i - 1])) {
      PointType point;
      point.normal_x = 0;
      point.normal_y = 0;
      point.normal_z = 0;
      point.x = pi.x;
      point.y = pi.y;
      point.z = pi.z;
      point.intensity = pi.intensity;
      point.curvature = (pi.timestamp - time_begin) * 1e-6;  // ns -> ms
      cloud_out->push_back(point);
    }
  }
}

void PointCloudPreprocess::ProcessOuster(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    pcl::PointCloud<PointType>::Ptr& cloud_out) {
  pcl::PointCloud<OusterPointXYZIRT> cloud_origin;
  pcl::fromROSMsg(*msg, cloud_origin);

  if (cloud_origin.empty()) return;
  // Anchor the per-point scan time (curvature) to the first firing of the scan.
  // For a native STAGGERED cloud, points[0] is already the first firing, so
  // t_start = points[0].t (single pass, original behavior).
  // For a DESTAGGERED cloud the rows are rolled per-beam, so points[0] lands ~1
  // column-period into the scan; points[0].t would make ~97% of curvatures
  // negative and corrupt deskew. Then anchor to min(t) instead (one extra pass).
  uint32_t t_start = cloud_origin.points[0].t;
  if (config_.ouster_destaggered) {
    for (const auto& p : cloud_origin.points) {
      t_start = std::min(t_start, p.t);
    }
  }

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
      // ms
      // point.curvature = cloud_origin.at(i).t * 1000;
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
