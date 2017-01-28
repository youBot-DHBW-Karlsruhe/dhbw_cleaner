#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "visualization_msgs/Marker.h"
#include "cleaner_alpha/Youbot.h"

class Cleaner {
private:
    ros::Subscriber nearestPointSubscriber;
    geometry_msgs::Point32 point;


public:
    Cleaner(ros::NodeHandle n, std::string nearestPointTopic) {
        nearestPointSubscriber = n.subscribe<geometry_msgs::Point32>(nearestPointTopic, 10, &Cleaner::nearestPointCallback, this);
    }

    void nearestPointCallback(const geometry_msgs::Point32::ConstPtr &p) {
        geometry_msgs::Point32 nearest;
        nearest.x = p.get()->x;
        nearest.y = p.get()->y;
        nearest.z = 0.0;
        ROS_INFO("Point receiverd");
        point = nearest;
    }

    geometry_msgs::Point32 nearestPoint() {
        return point;
    }

};


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

  std::stringstream ss;
  ss << "nearest_object_" << scan_cloud_topic.erase(0, 1);
  Cleaner c(n, ss.str().c_str());
  ros::spinOnce();

  youbot_proxy::Youbot youbot(youbot_proxy::Youbot::DEFAULT_POINT_SECONDS, youbot_proxy::Youbot::DEFAULT_SPEED);
  if(!youbot.initialize(n)) {
      ROS_ERROR("Could not initialize youbot proxy application. The driver may not be started yet?");
      return 1;
  }
  ROS_INFO("----- youbot proxy running");


  ROS_INFO("----- Cleaner running");
  //while(ros::ok()) {
      ros::spinOnce();
      geometry_msgs::Point32 p = c.nearestPoint();
      std::stringstream ss2;
      ss2 << "Next Point: x=" << p.x << ", y=" << p.y;
      ROS_INFO(ss2.str().c_str());


      // reduce distance and move base directly to the found point
      if(p.x == 0 && p.y == 0) {
          ROS_ERROR("Point was (0/0)!!");
          youbot.returnToInitPose();
          return 1;
      }
      p.x = p.x - 0.05; // -5cm
      p.y = p.y - 0.05; // -5cm
      youbot.moveBase(p.x, -p.y);


      // alternative 1: 1. rotate base -> 2. move base forward
      // alternative 2: feedback-loop: move to direction, check nearest object, correct movement, until object directly in front of youbot

      ros::spinOnce();
  //    loopRate.sleep();
  //}

  youbot.returnToInitPose();
  return 0;
}
