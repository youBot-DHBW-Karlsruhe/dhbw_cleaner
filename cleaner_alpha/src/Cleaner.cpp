#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include "cleaner_alpha/Youbot.h"
#include "object_finder_2d/NearestPoint.h"

class Cleaner {
private:
    ros::ServiceClient nearestPointService;
    geometry_msgs::Point32 point;


public:
    Cleaner(ros::NodeHandle n) {
        ros::service::waitForService("nearest_point");
        nearestPointService = n.serviceClient<object_finder_2d::NearestPoint>("nearest_point");
        ROS_INFO("Cleaner: Initialized");
    }

    geometry_msgs::Point32 nearestPoint() {
        object_finder_2d::NearestPoint srv;
        srv.request.command = "GET";
        if(nearestPointService.call(srv)) {
            return srv.response.point;
        } else {
            ROS_ERROR("Cleaner: Request to nearest_point service failed");
            return geometry_msgs::Point32();
        }
    }

};


//----------------------------------- MAIN -----------------------------------------------
int main(int argc, char** argv)
{
  // init node
  ros::init(argc, argv, "cleaner");
  ros::NodeHandle n;

  Cleaner c(n);
  youbot_proxy::Youbot youbot(youbot_proxy::Youbot::DEFAULT_POINT_SECONDS, youbot_proxy::Youbot::DEFAULT_SPEED);
  if(!youbot.initialize(n)) {
      ROS_ERROR("Could not initialize youbot proxy application. The driver may not be started yet?");
      return 1;
  }
  ROS_INFO("----- youbot proxy and cleaner running");
  ROS_INFO("--------------------------------------");


  // assuming that youbot base_link is at (0/0)
  // target: object at (0/0.5)
  geometry_msgs::Point32 goalPosition;
  goalPosition.x = 0.5;
  goalPosition.y = 0;
  geometry_msgs::Point32 p = c.nearestPoint();
  double tol = 0.03;

  // print point coordinates
  ROS_INFO_STREAM("Next Point: x=" << p.x << ", y=" << p.y);

  if(std::abs(p.x) <= tol && std::abs(p.y) <= tol) {
      ROS_ERROR("Point was nearly (0/0)!!");
      youbot.returnToInitPose();
      return 1;
  }

  // reduce distance and move base directly to the found point
  p.x = p.x - (goalPosition.x + 0.2);
  p.y = p.y - goalPosition.y;
  youbot.moveBase(p.x, p.y);

  // correct position to fit a good grabbing position iteratively
  bool grabPositionReached = false;
  geometry_msgs::Point32 objectPos = c.nearestPoint();
  while(!grabPositionReached && ros::ok()) {
      double angle, diagMovement;

      if(objectPos.x < 0) {
          ROS_ERROR("Object is behind the youBot?!");
          // should not be possible
          // in this case to following algorithm will not work
          return 1;
      }

      // turn base into direction of the object
      // - -> left
      // + -> right
      angle = std::asin(objectPos.y/objectPos.x);
      youbot.turnBaseRad(angle);

      // move base straight forward towards the object
      diagMovement = std::sqrt(std::pow(objectPos.x, 2) + std::pow(objectPos.y, 2)) - goalPosition.x;
      youbot.moveBase(diagMovement, goalPosition.y);

      // check position
      objectPos = c.nearestPoint();
      if(std::abs(objectPos.y) <= tol && objectPos.x >= goalPosition.x-tol && objectPos.x <= goalPosition.x+tol) {
          grabPositionReached = true;
      }

      ROS_INFO_STREAM("Next Point: x=" << objectPos.x << ", y=" << objectPos.y << ": reached: " << grabPositionReached);
}

  ros::Duration(1).sleep();

  // just move slightly forward
  youbot.moveBase(0.05, 0.0);

  ros::Duration(1).sleep();
  //TODO: insert object detection or position check with cemara here!

  // grab the object
  youbot.grab();
  youbot.drop();


  //TODO: later on: build a loop

  // exit cleaner-mode and return youBot to initial pose
  youbot.returnToInitPose();
  return 0;
}
