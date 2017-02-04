#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include "cleaner_alpha/Youbot.h"

class Cleaner {
private:
    ros::Subscriber nearestPointSubscriber;
    geometry_msgs::Point32 point;


public:
    Cleaner(ros::NodeHandle n, std::string nearestPointTopic) {
        nearestPointSubscriber = n.subscribe<geometry_msgs::Point32>(nearestPointTopic, 10, &Cleaner::nearestPointCallback, this);
        ROS_INFO("Cleaner: Initialized");
    }

    void nearestPointCallback(const geometry_msgs::Point32::ConstPtr &p) {
        geometry_msgs::Point32 nearest;
        nearest.x = p.get()->x;
        nearest.y = p.get()->y;
        nearest.z = 0.0;
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
  ros::Rate loopRate(ros::Duration(10));

  Cleaner c(n, "nearest_object");
  ros::spinOnce();

  youbot_proxy::Youbot youbot(youbot_proxy::Youbot::DEFAULT_POINT_SECONDS, youbot_proxy::Youbot::DEFAULT_SPEED);
  if(!youbot.initialize(n)) {
      ROS_ERROR("Could not initialize youbot proxy application. The driver may not be started yet?");
      return 1;
  }
  ROS_INFO("----- youbot proxy and cleaner running");
  ROS_INFO("--------------------------------------");

  /*
  int i = 2;
  while(ros::ok() && i-- > 0) {
  */
      // execute callbacks to retrieve nearest point
      ros::spinOnce();
      geometry_msgs::Point32 p = c.nearestPoint();
      std::stringstream ss;
      ss << "Next Point: x=" << p.x << ", y=" << p.y;
      ROS_INFO(ss.str().c_str());

      // reduce distance and move base directly to the found point
      if(p.x == 0 && p.y == 0) {
          ROS_ERROR("Point was (0/0)!!");
          youbot.returnToInitPose();
          return 1;
      }
      p.x = p.x - 0.1; // -10cm
      p.y = p.y - 0.1; // -10cm
      // laser is mounted upside-down, therefore change direction in y
      youbot.moveBase(p.x, -p.y);

      ros::spinOnce();
      p = c.nearestPoint();
      // calculate relative positions
      // assuming that youbot is at (0/0)
      // target: object at (0/5)
      double dx = p.x - 5;
      double dy = -p.y;
      youbot.moveBase(dx, dy);


      //youbot.grab();
      //youbot.drop();


      /*
      loopRate.reset();
      loopRate.sleep();
  }
  */

  youbot.returnToInitPose();
  return 0;
}
