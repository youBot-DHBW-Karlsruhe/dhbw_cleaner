#ifndef OBJECTFINDER_H
#define OBJECTFINDER_H

#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "visualization_msgs/Marker.h"
#include "object_finder_2d/NearestPoint.h"

namespace cleaner {

class ObjectFinder {

private:

  std::string baseLink;
  geometry_msgs::Point32 nearest;
  sensor_msgs::PointCloud cloud;
  visualization_msgs::Marker nearestPointMarker;

  ros::ServiceClient scanSrvClient;
  ros::ServiceServer nearestService;

  ros::Publisher cloudPublisher;
  ros::Publisher markerPublisher;

  void initMarker();

  geometry_msgs::Point32 extractNearestPoint(sensor_msgs::PointCloud::_points_type& points, double tol = 0.02);

  void printPoint(const geometry_msgs::Point32 point);

  void publishMarker(const geometry_msgs::Point32 point);

  bool invalidPointValue(const geometry_msgs::Point32 &point, double tol);

  double euclDist(const geometry_msgs::Point32 &point);



public:

  ObjectFinder(ros::NodeHandle n, std::string cloud_topic, std::string base_link);

  bool nearestPointServiceHandler(object_finder_2d::NearestPoint::Request &req, object_finder_2d::NearestPoint::Response &res);

  void publishIntermediates();
};

}

#endif // OBJECTFINDER_H
