#include "ros/ros.h"
#include "cleaner_alpha/ObjectFinder.h"

//----------------------------------- MAIN -----------------------------------------------
int main(int argc, char** argv)
{
  // init node
  ros::init(argc, argv, "scan_to_cloud");
  ros::NodeHandle n;

  // retrieve parameters
  std::string scan_cloud_topic, tf_base_link;
  ros::param::param<std::string>("transform_cloud_topic", scan_cloud_topic, "/laser_point_cloud");
  ros::param::param<std::string>("base_link", tf_base_link, "base_link");

  ROS_INFO("Using %s as scan_cloud_topic", scan_cloud_topic.c_str());
  ROS_INFO("Using %s as base_link", tf_base_link.c_str());

  cleaner::ObjectFinder objectFinder(n, scan_cloud_topic, tf_base_link);
  ROS_INFO("object finder running");
  ros::spin();
  
  return 0;
}
