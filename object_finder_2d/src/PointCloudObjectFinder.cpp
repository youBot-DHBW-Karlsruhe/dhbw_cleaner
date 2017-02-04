#include "ros/ros.h"
#include "cleaner_alpha/ObjectFinder.h"

//----------------------------------- MAIN -----------------------------------------------
int main(int argc, char** argv)
{
  // init node
  ros::init(argc, argv, "object_finder");
  ros::NodeHandle n;

  // retrieve parameters
  std::string scan_topic, cloud_topic, tf_base_link;
  bool debug;
  ros::param::param<std::string>("laser_scan_topic", scan_topic, "/laser_scan");
  ros::param::param<std::string>("transform_cloud_topic", cloud_topic, "/laser_point_cloud");
  ros::param::param<std::string>("base_link", tf_base_link, "base_link");
  ros::param::param<bool>("debugging", debug, false);

  ROS_INFO("Using %s as scan_cloud_topic", scan_topic.c_str());
  ROS_INFO("Using %s as transform_cloud_topic", cloud_topic.c_str());
  ROS_INFO("Using %s as base_link", tf_base_link.c_str());
  if(debug)
      ROS_INFO("debugging is enabled");

  // initialize classes
  cleaner::ObjectFinder objectFinder(n, cloud_topic, tf_base_link);

  ROS_INFO("--- point cloud object finder running");

  if(debug) {
      while(ros::ok()) {
          // publish debug output for rviz
          objectFinder.publishIntermediates();
          ros::Duration(1).sleep();
          ros::spinOnce();
      }
  } else {
      ros::spin();
  }
  
  return 0;
}
