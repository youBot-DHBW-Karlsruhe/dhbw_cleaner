#include "ros/ros.h"
#include "cleaner_alpha/ObjectFinder.h"
#include "cleaner_alpha/LaserScanToPointCloud.h"

//----------------------------------- MAIN -----------------------------------------------
int main(int argc, char** argv)
{
  // init node
  ros::init(argc, argv, "scan_to_cloud");
  ros::NodeHandle n;

  // retrieve parameters
  std::string scan_topic, cloud_topic, tf_base_link;
  ros::param::param<std::string>("laser_scan_topic", scan_topic, "/laser_scan");
  ros::param::param<std::string>("transform_cloud_topic", cloud_topic, "/laser_point_cloud");
  ros::param::param<std::string>("base_link", tf_base_link, "base_link");

  ROS_INFO("Using %s as scan_cloud_topic", scan_topic.c_str());
  ROS_INFO("Using %s as base_link", tf_base_link.c_str());

  cleaner::LaserScanToPointCloud scanTransform(n, scan_topic, cloud_topic, tf_base_link);
  cleaner::ObjectFinder objectFinder(n, tf_base_link);
  scanTransform.registerCallback("nearestPointExtractor", boost::bind(&cleaner::ObjectFinder::receivePointCloud, objectFinder, _1));
  ROS_INFO("object finder running");

  ros::spin();
  
  return 0;
}
