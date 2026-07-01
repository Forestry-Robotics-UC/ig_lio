/*
 * @Description: preprocess point clouds
 * @Autor: Zijie Chen
 * @Date: 2023-12-28 09:45:32
 */

#ifndef POINTCLOUD_PREPROCESS_H_
#define POINTCLOUD_PREPROCESS_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <glog/logging.h>

#include <pcl_conversions/pcl_conversions.h>

#ifdef HAVE_LIVOX
#include <livox_ros_driver2/CustomMsg.h>
#endif

#include "point_type.h"

enum class LidarType { VELODYNE, OUSTER, HESAI, VELODYNEM1600, LIVOX, LIVOX_POINTS };

// for Livox LiDARs published as PointCloud2 (livox_ros_driver2 xfer_format=0),
// e.g. a Mid-360. Layout: x,y,z,intensity (float), tag,line (uint8),
// timestamp (double, absolute time of the point in nanoseconds).
struct LivoxPointXYZITLT {
  PCL_ADD_POINT4D;
  float intensity;
  std::uint8_t tag;
  std::uint8_t line;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    LivoxPointXYZITLT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        std::uint8_t, tag, tag)(std::uint8_t, line, line)(double, timestamp, timestamp))

// for Velodyne LiDAR
struct VelodynePointXYZIRT {
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    VelodynePointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        uint16_t, ring, ring)(float, time, t))
struct VelodyneM1600PointXYZIRT {
  PCL_ADD_POINT4D;
  uint8_t intensity;
  uint8_t ring;
  uint32_t timestampSec;
  uint32_t timestampNsec;
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    VelodyneM1600PointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(
        uint8_t, ring, ring)(uint32_t, timestampSec, timestampSec)(uint32_t, timestampNsec, timestampNsec))
        

struct HesaiPointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;
  double timestamp;
  uint16_t ring;
  // Add any additional fields specific to Hesai LiDAR
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    HesaiPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(
        uint16_t, ring, ring))

// for Ouster LiDAR
struct OusterPointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;
  uint16_t reflectivity;
  uint16_t ring;
  uint16_t noise;
  uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    OusterPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        uint32_t, t, t)(uint16_t, reflectivity, reflectivity)(
        uint16_t, ring, ring)(uint16_t, noise, ambient)(uint32_t, range, range))
// struct OusterPointXYZIRT {
//   PCL_ADD_POINT4D;
//   float intensity;
//   double timestamp;
//   uint16_t reflectivity;
//   uint8_t ring;
//   uint16_t noise;
//   uint32_t range;
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// } EIGEN_ALIGN16;
// POINT_CLOUD_REGISTER_POINT_STRUCT(
//     OusterPointXYZIRT,
//     (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
//         double, timestamp, timestamp)(uint16_t, reflectivity, reflectivity)(
//         uint8_t, ring, ring)(uint16_t, noise, noise)(uint32_t, range, range))

class PointCloudPreprocess {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  struct Config {
    Config(){};

    int point_filter_num{4};
    LidarType lidar_type = LidarType::VELODYNE;
    // only use for velodyne
    double time_scale{1000.0};
    // Ouster only: if the incoming cloud is destaggered (columns rolled per
    // beam so points[0] is no longer the first firing), anchor the per-point
    // scan time to the minimum t over the cloud instead of points[0].t. Costs
    // one extra pass; leave false for native staggered clouds (the default) to
    // keep the original single-pass behavior. See ProcessOuster.
    bool ouster_destaggered{false};
  };

  PointCloudPreprocess() = delete;

  PointCloudPreprocess(Config config = Config())
      : config_(config) {}

  ~PointCloudPreprocess() = default;

  void Process(const sensor_msgs::PointCloud2::ConstPtr& msg,
               pcl::PointCloud<PointType>::Ptr& cloud_out);

#ifdef HAVE_LIVOX
  // Livox arrives as CustomMsg (not PointCloud2), so it has its own entry point.
  // Ported from upstream; NOT validated on hardware in this fork.
  void ProcessLivox(const livox_ros_driver2::CustomMsg::ConstPtr& msg,
                    pcl::PointCloud<PointType>::Ptr& cloud_out);
#endif

 private:
  template <typename T>
  bool HasInf(const T& p);

  template <typename T>
  bool HasNan(const T& p);

  template <typename T>
  bool IsNear(const T& p1, const T& p2);

  void ProcessVelodyne(const sensor_msgs::PointCloud2::ConstPtr& msg,
                       pcl::PointCloud<PointType>::Ptr& cloud_out);
  void ProcessVelodyneM1600(const sensor_msgs::PointCloud2::ConstPtr& msg,
                       pcl::PointCloud<PointType>::Ptr& cloud_out);
  void ProcessHesai(const sensor_msgs::PointCloud2::ConstPtr& msg,
                  pcl::PointCloud<PointType>::Ptr& cloud_out);

  void ProcessOuster(const sensor_msgs::PointCloud2::ConstPtr& msg,
                     pcl::PointCloud<PointType>::Ptr& cloud_out);

  // Livox published as PointCloud2 (e.g. Mid-360), as opposed to CustomMsg.
  void ProcessLivoxPointCloud2(const sensor_msgs::PointCloud2::ConstPtr& msg,
                               pcl::PointCloud<PointType>::Ptr& cloud_out);

  int num_scans_ = 128;
  bool has_time_ = false;

  Config config_;

  pcl::PointCloud<PointType> cloud_sort_;
};

#endif
