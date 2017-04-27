#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include "tf/transform_datatypes.h"

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
    ros::Subscriber objectPositionSingleSubscriber;
    ros::Subscriber objectPositionAggregatedSubscriber;

    geometry_msgs::Pose objectPosition_single;
    geometry_msgs::Pose objectPosition_aggregated;
    bool available, found;

public:
    ObjectDetectionListener(ros::NodeHandle& node, std::string objectPositionTopic = "object_position_single", std::string objectPositionAggregatedTopic = "object_position_aggregated") {
        objectPositionSingleSubscriber = node.subscribe<object_recognition::ObjectPosition>(objectPositionTopic, 5, &ObjectDetectionListener::objectPosition_single_Callback, this);
        objectPositionAggregatedSubscriber = node.subscribe<object_recognition::ObjectPosition>(objectPositionTopic, 5, &ObjectDetectionListener::objectPosition_aggregated_Callback, this);
        available = false;
        found = false;
    }

    void objectPosition_single_Callback(const object_recognition::ObjectPositionConstPtr& msg) {
        geometry_msgs::Pose pose;
        pose.position = msg->pose.position;
        pose.orientation = msg->pose.orientation;
        objectPosition_single = pose;
        found = true;
    }


    void objectPosition_aggregated_Callback(const object_recognition::ObjectPositionConstPtr& msg) {
        geometry_msgs::Pose pose;
        pose.position = msg->pose.position;
        pose.orientation = msg->pose.orientation;
        objectPosition_aggregated = pose;
        available = true;
    }

    geometry_msgs::Pose getObjectPosition() {
        available = false;
        return objectPosition_aggregated;
    }

    bool foundObject() {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ros::spinOnce();
        bool result = found;
        found = false;
        return result;
    }

    bool isAvailable() {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ros::spinOnce();
        return available;
    }
};

//----------------------------------- GLOBAL SCOPE ----------------------------
// global Timeout
const ros::Duration TIMEOUT(30);

bool timedOut(ros::Time startTime, const ros::Duration duration = TIMEOUT) {
    return ros::Time::now() - startTime >= duration;
}

void terminate_timeout(std::string msg) {
    ROS_ERROR_STREAM("Timed out: " << msg << " (Timeout=" << TIMEOUT.sec << "s)");
    ros::shutdown();
    exit(1);
}

void waitForValidNearestPoint(NearestPointServiceClient& service, geometry_msgs::Point32* point) {
    double tol = 0.03;
    ros::Time startTime = ros::Time::now();
    ros::Rate loopRate(5);
    *point = service.nearestPoint();

    while(std::abs(point->x) <= tol && std::abs(point->y) <= tol && ros::ok()) {
        ROS_WARN_STREAM("Point was nearly (0/0), tolerance=" << tol << "!!");
        ros::spinOnce();
        loopRate.sleep();
        *point = service.nearestPoint();

        if(timedOut(startTime)) {
            terminate_timeout("No valid point received.");
        }
    }
}

bool checkSelfcollision(const geometry_msgs::Pose& objectPose) {
    int x = objectPose.position.x;
    int y = objectPose.position.y;
    // relative to frame arm_link_0
    int baseWidth = 0.39;
    int baseLength = 0.570;
    int timWidth = 0.07;
    int timLength = 0.08;
    int arm0_x = 0.135;
    int arm0_y = 0;

    // sick tim collision
    if(x < baseLength/2.0 + timLength - arm0_x && x >= baseLength/2.0 - arm0_x &&
       std::abs(y) <= timWidth) {
        return true;
    }

    // base front
    if(x < baseLength/2.0 - arm0_x && x > baseLength/2.0 + arm0_x &&
       std::abs(y) <= baseWidth/2.) {
        return true;
    }

    return false;
}

bool checkInRange(const geometry_msgs::Pose& objectPose) {
    int x = objectPose.position.x;
    int y = objectPose.position.y;
    // relative to frame arm_link_0

    if(0.32 <= std::sqrt(x*x + y*y)) {
        return false;
    }
    return true;
}

geometry_msgs::Quaternion normalizeOrientation(const geometry_msgs::Pose& objectPose) {
    double roll, pitch, yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(objectPose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    ROS_INFO_STREAM("Object Position:\n x=" << objectPose.position.x << ", y=" << objectPose.position.y << ", z=" << objectPose.position.z);
    ROS_INFO_STREAM("Object Orientation:\n Roll=" << roll << ", Pitch=" << pitch << ", Yaw=" << yaw);

    pitch = 0;
    yaw = 0;
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    return q;
}


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
    ObjectDetectionListener objectDetection(n, "object_position_single");
    youbot_proxy::YoubotBase youbot(n, "/cmd_vel", youbot_proxy::YoubotBase::DEFAULT_POINT_SECONDS, 0.1);
    youbot_proxy::TrajectoryGeneratorFactory tgFactory(n);
    youbot_proxy::Gripper gripper(n, "/arm_1/gripper_controller/position_command");
    youbot_proxy::Manipulator manipulator(n, gripper, tgFactory, "/joint_states", "/torque_control");

    ROS_INFO("----- youbot proxy and cleaner running");
    ROS_INFO("--------------------------------------");

    // run this node
    // ------------------------------------------------------------------------
    // assuming that youbot base_link is at (0/0)
    // target: object at (0/0.38)
    geometry_msgs::Point32 goalPosition;
    goalPosition.x = 0.39;
    goalPosition.y = 0;
    geometry_msgs::Point32 p;
    // waiting time until object detection will cancel
    ros::Duration maxDetectionDuration(20, 0);

    // print point coordinates
    waitForValidNearestPoint(nearestPointService, &p);
    ROS_INFO_STREAM("Next Point: x=" << p.x << ", y=" << p.y);

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
    angle = std::atan2(p.y, p.x);
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
      ros::Time startWaiting = ros::Time::now();
      while(!objectDetection.foundObject() && ros::ok()) {
          if(timedOut(startWaiting, maxDetectionDuration)) {
              ROS_WARN("No grabbable object found!");
              terminate_timeout("no logic implemented for this case");
          }
      }
      while(!objectDetection.isAvailable() && ros::ok()) {};
      geometry_msgs::Pose objectPose = objectDetection.getObjectPosition();
      ROS_INFO("... received object coordinates");


      // check orientation values
      objectPose.orientation = normalizeOrientation(objectPose);

      // check position values
      if(checkSelfcollision(objectPose)) {
          ROS_WARN("Grab position would selfcollide");
          return 1;
      }
      if(!checkInRange(objectPose)) {
          ROS_WARN("Grab position outside of gripper range");
          return 1;
      }

      if(manipulator.grabObjectAt(objectPose)) {
          ROS_ERROR("main(): grabbing successful");

          ros::spinOnce();
          ros::Duration(1.0).sleep();


          // drop object on plate
          // ------------------------------------------------------------------
          if(!manipulator.dropObject()) {
              ROS_ERROR("Could not move arm to DROP pose!");
              return 1;
          }
          if(!manipulator.moveArmToPose(youbot_proxy::util::Pose::OBSERVE_FAR)) {
              ROS_ERROR("Could not move arm to OBSERVE pose!");
              return 1;
          }
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

}
