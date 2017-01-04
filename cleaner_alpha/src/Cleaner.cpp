#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "visualization_msgs/Marker.h"
#include "cleaner_alpha/Youbot.h"
#include "cleaner_alpha/ObjectFinder.h"

//----------------------------------- MAIN -----------------------------------------------
int main(int argc, char** argv)
{
  // init node
  ros::init(argc, argv, "cleaner");
  ros::NodeHandle n;
  ros::Rate loopRate(ros::Duration(2));

  // retrieve ros parameters
  std::string scan_cloud_topic, tf_base_link;
  ros::param::param<std::string>("transform_cloud_topic", scan_cloud_topic, "/laser_point_cloud");
  ros::param::param<std::string>("base_link", tf_base_link, "base_link");

  ROS_INFO("Using %s as transform_cloud_topic", scan_cloud_topic.c_str());
  ROS_INFO("Using %s as base_link", tf_base_link.c_str());

  cleaner::ObjectFinder objectFinder(n, scan_cloud_topic, tf_base_link);
  ROS_INFO("----- object finder running");

  youbot_proxy::Youbot youbot(youbot_proxy::Youbot::DEFAULT_POINT_SECONDS, youbot_proxy::Youbot::DEFAULT_SPEED);
  if(!youbot.initialize(n)) {
      objectFinder.publishNearestPoint();
      objectFinder.printNearestPoint();
      ROS_ERROR("Could not initialize youbot proxy application. The driver may not be started yet?");
      return 1;
  }
  ROS_INFO("----- youbot proxy running");

  ROS_INFO("Cleaner running");
  while(ros::ok()) {
      geometry_msgs::Point32 p = objectFinder.nearestPoint();
      objectFinder.publishNearestPoint();
      objectFinder.printNearestPoint();

      // reduce distance and move base directly to the found point
      p.x = p.x - 0.05; // -5cm
      p.y = p.y - 0.05; // -5cm
      youbot.moveBase(p.x, p.y);
      // alternative 1: 1. rotate base -> 2. move base forward
      // alternative 2: feedback-loop: move to direction, check nearest object, correct movement, until object directly in front of youbot

      ros::spinOnce();
      loopRate.sleep();
  }

  return 0;
}
