#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include "object_finder_2d/NearestPoint.h"
#include "object_recognition/ObjectPosition.h"
#include "cleaner_alpha/YoubotBase.h"
#include "cleaner_alpha/Manipulator.h"
#include "cleaner_alpha/Gripper.h"


class NearestPointServiceClient {
private:
    ros::ServiceClient nearestPointService;
    geometry_msgs::Point32 point;


public:
    NearestPointServiceClient(ros::NodeHandle& n, std::string nearestPointService_name = "nearest_point") {
        ros::service::waitForService(nearestPointService_name);
        nearestPointService = n.serviceClient<object_finder_2d::NearestPoint>(nearestPointService_name);
        ROS_INFO("NearestPointServiceClient: Initialized");
    }

    geometry_msgs::Point32 nearestPoint() {
        object_finder_2d::NearestPoint srv;
        srv.request.command = "GET";
        if(nearestPointService.call(srv)) {
            return srv.response.point;
        } else {
            ROS_ERROR("NearestPointServiceClient: Request to nearest_point service failed");
            return geometry_msgs::Point32();
        }
    }

};

class ObjectDetectionListener {
private:
    ros::Subscriber objectPositionSubscriber;

    geometry_msgs::Pose objectPosition;
    bool available;

public:
    ObjectDetectionListener(ros::NodeHandle& node, std::string objectPositionTopic = "object_position_single") {
        objectPositionSubscriber = node.subscribe<object_recognition::ObjectPosition>(objectPositionTopic, 5, &ObjectDetectionListener::objectPositionCallback, this);
        available = false;
    }

    void objectPositionCallback(const object_recognition::ObjectPositionConstPtr& msg) {
        //if(msg->object_id != "") {
        //    return;
        //}
        geometry_msgs::Pose pose;
        pose.position = msg->pose.position;
        pose.orientation = msg->pose.orientation;
        objectPosition = pose;
        available = true;
    }

    geometry_msgs::Pose getObjectPosition() {
        available = false;
        return objectPosition;
    }

    bool isAvailable() {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ros::spinOnce();
        return available;
    }
};


//----------------------------------- MAIN -----------------------------------------------
int main(int argc, char** argv)
{
  // init node
  ros::init(argc, argv, "cleaner");
  ros::NodeHandle n;

  // initialize classes
  NearestPointServiceClient nearestPointService(n, "nearest_point");
  ObjectDetectionListener objectDetection(n, "object_position_aggregated");
  youbot_proxy::YoubotBase youbot(n, "/cmd_vel", youbot_proxy::YoubotBase::DEFAULT_POINT_SECONDS, youbot_proxy::YoubotBase::DEFAULT_SPEED);
  youbot_proxy::TrajectoryGeneratorFactory tgFactory(n);
  youbot_proxy::Gripper gripper(n, "/arm_1/gripper_controller/position_command");
  youbot_proxy::Manipulator manipulator(n, gripper, tgFactory, "/joint_states", "/torque_control");

  ROS_INFO("----- youbot proxy and cleaner running");
  ROS_INFO("--------------------------------------");


  // assuming that youbot base_link is at (0/0)
  // target: object at (0/0.5)
  geometry_msgs::Point32 goalPosition;
  goalPosition.x = 0.5;
  goalPosition.y = 0;
  geometry_msgs::Point32 p = nearestPointService.nearestPoint();
  double tol = 0.03;

  // print point coordinates
  ROS_INFO_STREAM("Next Point: x=" << p.x << ", y=" << p.y);

  if(std::abs(p.x) <= tol && std::abs(p.y) <= tol) {
      ROS_ERROR("Point was nearly (0/0)!!");
      return 1;
  }

  // reduce distance and move base directly to the found point
  p.x = p.x - (goalPosition.x + 0.2);
  p.y = p.y - goalPosition.y;
  youbot.move(p.x, p.y);

  // correct position to fit a good grabbing position iteratively
  bool grabPositionReached = false;
  geometry_msgs::Point32 objectPos = nearestPointService.nearestPoint();
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
      youbot.turnRad(angle);

      // move base straight forward towards the object
      diagMovement = std::sqrt(std::pow(objectPos.x, 2) + std::pow(objectPos.y, 2)) - goalPosition.x;
      youbot.move(diagMovement, goalPosition.y);

      // check position
      objectPos = nearestPointService.nearestPoint();
      if(std::abs(objectPos.y) <= tol && objectPos.x >= goalPosition.x-tol && objectPos.x <= goalPosition.x+tol) {
          grabPositionReached = true;
      }

      ROS_INFO_STREAM("Next Point: x=" << objectPos.x << ", y=" << objectPos.y << ": reached: " << grabPositionReached);
}

  ros::Duration(1).sleep();

  // just move slightly forward
  youbot.move(0.05, 0.0);

  ros::Duration(1).sleep();
  //TODO: insert object detection or position check with cemara here!

  // grab the object
  /*
  manipulator.grab();
  manipulator.drop();
  */

  //TODO: later on: build a loop

  // exit cleaner-mode and return youBot to initial pose
  return 0;
}
