#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include "tf/transform_datatypes.h"

#include "object_finder_2d/NearestPoint.h"
#include "object_recognition/ObjectPosition.h"

#include "cleaner_alpha/YoubotBase.h"
#include "cleaner_alpha/Manipulator.h"
#include "cleaner_alpha/Gripper.h"

// for testing only!
#include <iostream>


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
    // initialize everything
    // ------------------------------------------------------------------------

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

    // run this node
    // ------------------------------------------------------------------------
    // assuming that youbot base_link is at (0/0)
    // target: object at (0/0.4)
    geometry_msgs::Point32 goalPosition;
    goalPosition.x = 0.4;
    goalPosition.y = 0;
    geometry_msgs::Point32 p = nearestPointService.nearestPoint();
    double tol = 0.03;

    // print point coordinates
    ROS_INFO_STREAM("Next Point: x=" << p.x << ", y=" << p.y);

    if(std::abs(p.x) <= tol && std::abs(p.y) <= tol) {
      ROS_ERROR_STREAM("Point was nearly (0/0), tolerance=" << tol << "!!");
      return 1;
    }
    if(p.x < 0) {
      ROS_ERROR("Object is behind the youBot?!");
      // should not be possible
      // in this case to following algorithm will not work
      return 1;
    }


    // move base to the nearest object
    // ------------------------------------------------------------------------
    double angle, diagMovement;

    // turn base into direction of the object
    // - -> left
    // + -> right
    angle = std::asin(p.y/p.x);
    youbot.turnRad(angle);
    // move base straight forward towards the object
    diagMovement = std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2)) - goalPosition.x;
    youbot.move(diagMovement, goalPosition.y);

    // check position
    p = nearestPointService.nearestPoint();
    ROS_INFO_STREAM("Nearest Point after movement: x=" << p.x << ", y=" << p.y);


    // unfold arm
    // ------------------------------------------------------------------------
    if(!manipulator.moveArmToPose(youbot_proxy::util::Pose::OBSERVE_FAR)) {
      ROS_ERROR("Could not move arm to OBSERVE pose!");
      return 1;
    }
    manipulator.openGripper();


    // observe object position and orientation in preparation for grasping
    // and grab object
    // ------------------------------------------------------------------------
    while(ros::ok()) {
      // retrieve object position
      ROS_INFO("Waiting for object detection...");
      while(!objectDetection.isAvailable() && ros::ok()) {}
      geometry_msgs::Pose objectPose = objectDetection.getObjectPosition();
      ROS_INFO("... received object coordinates");


      // check orientation values
      double roll, pitch, yaw;
      tf::Quaternion quat;
      tf::quaternionMsgToTF(objectPose.orientation, quat);
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      ROS_INFO_STREAM("Object Position:\n x=" << objectPose.position.x << ", y=" << objectPose.position.y << ", z=" << objectPose.position.z);
      ROS_INFO_STREAM("Object Orientation:\n Roll=" << roll << ", Pitch=" << pitch << ", Yaw=" << yaw);


      if(pitch != 0) pitch = 0;
      if(yaw != 0) yaw = 0;
      geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
      objectPose.orientation = q;

      /*
        TODO: test for range before using expensive IK
        range in 0.31 > std:sqrt(x² + y²)
       */
      if(manipulator.grabObjectAt(objectPose)) {
          ROS_ERROR("main(): grabbing successful");


          // drop object on plate
          // ------------------------------------------------------------------
          if(!manipulator.moveArmToPose(youbot_proxy::util::Pose::DROP_AT_PLATE)) {
              ROS_ERROR("Could not move arm to DROP pose!");
              return 1;
          }
          manipulator.openGripper();
          ros::Duration(1.5).sleep();
          break;
      }
      ROS_ERROR("main(): grabbing failed");
    }

    // exit cleaner-mode and return youBot to initial pose
    // ------------------------------------------------------------------------
    if(!manipulator.returnToInit()) {
      ROS_ERROR("Could not move arm to INIT pose!");
    }
    return 0;


    //TODO: later on: build a loop
}

/*
// old code:


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
*/
