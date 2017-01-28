#ifndef OBJECTFINDER_H
#define OBJECTFINDER_H

#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "visualization_msgs/Marker.h"

namespace cleaner {

class ObjectFinder {

private:

  std::string baseLink;
  geometry_msgs::Point32 nearest;
  visualization_msgs::Marker nearestPointMarker;
  ros::Subscriber scanSubscriber;
  ros::Publisher markerPublisher;
  ros::Publisher nearestObjectPublisher;

  void initMarker();

  geometry_msgs::Point32 extractNearestPoint(sensor_msgs::PointCloud::_points_type points, double tol = 0.01);

  void printPoint(const geometry_msgs::Point32 point);

  void publishMarker(const geometry_msgs::Point32 point);

  bool invalidPointValue(const geometry_msgs::Point32 &point, double tol);

  double euclDist(const geometry_msgs::Point32 &point);



public:

  ObjectFinder(ros::NodeHandle n, std::string scan_topic, std::string base_link);

  void scanCallback(const sensor_msgs::PointCloud::ConstPtr& pointCloud);

};

}

#endif // OBJECTFINDER_H
