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

  void initMarker();

  geometry_msgs::Point32 extractNearestPoint(sensor_msgs::PointCloud::_points_type points, double tol = 0.01);

  void printPoint(const geometry_msgs::Point32 point);

  void publishPoint(const geometry_msgs::Point32 point);

  double euclDist(const geometry_msgs::Point32 &point);



public:

  ObjectFinder(ros::NodeHandle n, std::string scan_topic, std::string base_link);

  void scanCallback(const sensor_msgs::PointCloud::ConstPtr& pointCloud);

  /*void registerCallback(boost::function<void (geometry_msgs::Point32 np)> callback) {
      callbackFunction = callback;
  }*/

  geometry_msgs::Point32 nearestPoint();

  void publishNearestPoint();

  void printNearestPoint();

};

}

#endif // OBJECTFINDER_H
