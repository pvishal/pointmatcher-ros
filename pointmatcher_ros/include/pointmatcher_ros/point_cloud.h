#ifndef __POINTMATCHER_ROS_POINT_CLOUD_H
#define __POINTMATCHER_ROS_POINT_CLOUD_H

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/IO.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"

namespace ros
{
struct Time;
};
namespace tf
{
struct TransformListener;
};

namespace PointMatcher_ros
{
template <typename T>
typename PointMatcher<T>::DataPoints rosMsgToPointMatcherCloud(const sensor_msgs::PointCloud2 &rosMsg, const bool isDense = false);

template <typename T>
typename PointMatcher<T>::DataPoints rosMsgToPointMatcherCloud(const sensor_msgs::LaserScan &rosMsg, const bool addTimestamps = false);

template <typename T>
sensor_msgs::PointCloud2 pointMatcherCloudToRosMsg(const typename PointMatcher<T>::DataPoints &pmCloud, const std::string &frame_id, const ros::Time &stamp);
}; // namespace PointMatcher_ros

#endif //__POINTMATCHER_ROS_POINT_CLOUD_H
