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
        objectPositionSingleSubscriber = node.subscribe<object_recognition::ObjectPosition>(objectPositionTopic, 5, &ObjectDetectionListener::objectPosition_single_callback, this);
        objectPositionAggregatedSubscriber = node.subscribe<object_recognition::ObjectPosition>(objectPositionAggregatedTopic, 5, &ObjectDetectionListener::objectPosition_aggregated_callback, this);
        available = false;
        found = false;
    }

    void objectPosition_single_callback(const object_recognition::ObjectPositionConstPtr& msg) {
        geometry_msgs::Pose pose;
        pose.position = msg->pose.position;
        pose.orientation = msg->pose.orientation;
        objectPosition_single = pose;
        found = true;
    }


    void objectPosition_aggregated_callback(const object_recognition::ObjectPositionConstPtr& msg) {
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

    bool isAvailable() const {
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

    while(std::abs(point->x) <= tol &&
          std::abs(point->y) <= tol &&
          point->x < 0 &&
          ros::ok()) {
        ROS_WARN_STREAM("Point was nearly (0/0) or negative in x-achsis, tolerance=" << tol << "!!");
        ros::spinOnce();
        loopRate.sleep();
        *point = service.nearestPoint();

        if(timedOut(startTime)) {
            terminate_timeout("No valid point received.");
        }
    }
}

void moveToObjectAt(youbot_proxy::YoubotBase& youbot, const geometry_msgs::Point32& p, const geometry_msgs::Point32& goalPosition) {
    double angle, diagMovement;

    // turn base into direction of the object
    // - -> left
    // + -> right
    angle = std::atan2(p.y, p.x);
    youbot.turnRad(angle);

    // move base straight forward towards the object
    diagMovement = std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2)) - goalPosition.x;
    youbot.move(diagMovement, goalPosition.y);
}

bool detectObject(ObjectDetectionListener& objectDetection, ros::Duration maxDetectionDuration, geometry_msgs::Pose* objectPose) {
    ros::Time startWaiting;


    ROS_INFO("Waiting for object detection...");
    startWaiting = ros::Time::now();
    while(!objectDetection.foundObject() && ros::ok()) {
        if(timedOut(startWaiting, maxDetectionDuration)) {
            ROS_WARN("... no grabbable object found!");
            return false;
        }
    }

    while(!objectDetection.isAvailable() && ros::ok()) {
        if(timedOut(startWaiting, maxDetectionDuration)) {
            ROS_WARN("... found object, but could not retrieve aggregated position data");
            return false;
        }
    }
    *objectPose = objectDetection.getObjectPosition();
    ROS_INFO("... received object coordinates");
    return true;
}

bool checkSelfcollision(const geometry_msgs::Pose& objectPose) {
    double x = objectPose.position.x;
    double y = objectPose.position.y;
    // relative to frame arm_link_0
    double baseWidth = 0.39;
    double baseLength = 0.570;
    double timWidth = 0.07;
    double timLength = 0.08;
    double arm0_x = 0.135;
    double arm0_y = 0;

    // sick tim collision
    if(x < baseLength/2.0 + timLength - arm0_x &&
       x >= baseLength/2.0 - arm0_x &&
       std::abs(y) <= timWidth) {
        return true;
    }

    // base front
    if(x < baseLength/2.0 - arm0_x &&
       x > baseLength/2.0 + arm0_x &&
       std::abs(y) <= baseWidth/2.) {
        return true;
    }

    return false;
}

bool checkInRange(const geometry_msgs::Pose& objectPose) {
    double x = objectPose.position.x;
    double y = objectPose.position.y;
    // relative to frame arm_link_0

    return 0.32 <= std::sqrt(x*x + y*y);
}

bool checkIfPointIsNear(const geometry_msgs::Point32& point) {
    double x = point.x - 0.135; // distance from arm_link_0
    double y = point.y;

    return x < 0.32 && x > 0.23
            && std::abs(y) <=20;
}

geometry_msgs::Point32 calcPositionCorrection(const geometry_msgs::Point32& object) {
    geometry_msgs::Point32 movement;
    movement.x = 0;
    movement.y = (object.y > 0)? -0.02 : 0.02;

    // angular movement needed?
    return movement;
}

geometry_msgs::Quaternion normalizeOrientation(geometry_msgs::Pose* objectPose) {
    double roll, pitch, yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(objectPose->orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    ROS_INFO_STREAM("Object Position:\n x=" << objectPose->position.x << ", y=" << objectPose->position.y << ", z=" << objectPose->position.z);
    ROS_INFO_STREAM("Object Orientation:\n Roll=" << roll << ", Pitch=" << pitch << ", Yaw=" << yaw);

    pitch = 0;
    yaw = 0;
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    objectPose->orientation = q;
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
    youbot_proxy::Manipulator manipulator(n, gripper, tgFactory, "/joint_states", "/torque_control", "/arm_1/arm_controller/follow_joint_trajectory");

    // init variables
    enum State {
        FIND_NEXT_OBJECT, DRIVE_TO_OBJECT, DETECT_OBJECT, TRY_GRASP, CORRECT_POSITION, FINISHED, ERROR
    };

    int nObjects = 2;
    geometry_msgs::Point32 goalPosition;                    // assuming that youbot base_link is at (0/0),
    goalPosition.x = 0.39;                                  // then target: object at (0/0.39)
    goalPosition.y = 0;
    geometry_msgs::Point32 p;
    ros::Duration maxDetectionDuration(20, 0);              // waiting time until object detection will cancel
    geometry_msgs::Pose objectPose;
    State state = FIND_NEXT_OBJECT;
    int correctionAttempts = 0;

    ROS_INFO("----- youbot proxy and cleaner running");
    ROS_INFO("--------------------------------------");


    // run this node: main routine
    // ------------------------------------------------------------------------
    while(state != FINISHED && ros::ok()) {
        switch(state) {
        case FIND_NEXT_OBJECT:
            // find nearest point (--> object)
            waitForValidNearestPoint(nearestPointService, &p);
            ROS_INFO_STREAM("Next Point: x=" << p.x << ", y=" << p.y);

            state = DRIVE_TO_OBJECT;
            break;

        case DRIVE_TO_OBJECT:
            // arm to move pose?
            //TODO

            // move base to the nearest object
            moveToObjectAt(youbot, p, goalPosition);

            state = DETECT_OBJECT;
            break;

        case DETECT_OBJECT:
            // unfold arm
            if(!manipulator.moveArmToPose(youbot_proxy::util::Pose::OBSERVE_FAR)) {
                ROS_ERROR("Could not move arm to OBSERVE pose!");
                return 1;
            }

            // retrieve object position
            if(detectObject(objectDetection, maxDetectionDuration, &objectPose)) {
                state = TRY_GRASP;
            } else {
                state = CORRECT_POSITION;
            }
            break;

        case CORRECT_POSITION:
            // get position of nearest point
            waitForValidNearestPoint(nearestPointService, &p);

            // inside bounds?
            if(checkIfPointIsNear(p) && correctionAttempts < 3) {
                // correct position
                geometry_msgs::Point32 movement = calcPositionCorrection(p);
                youbot.move(movement.x, movement.y);
                correctionAttempts++;

                state = DETECT_OBJECT;
            } else {
                correctionAttempts = 0;
                state = DRIVE_TO_OBJECT;
            }
            break;

        case TRY_GRASP:
            // check position values
            normalizeOrientation(&objectPose);
            if(checkSelfcollision(objectPose)) {
                ROS_WARN("Grab position would selfcollide");
                // calc correction and back to detectObject-loop
                state = DETECT_OBJECT;
                break;
            }
            if(!checkInRange(objectPose)) {
                ROS_WARN("Grab position outside of gripper range");
                // calc correction and back to detectObject-loop
                state = DETECT_OBJECT;
                break;
            }

            // grab object
            if(!manipulator.grabObjectAt(objectPose)) {
                ROS_ERROR("Grabbing failed!");
                return 1;
            }

            // drop object on plate
            if(!manipulator.dropObject()) {
                ROS_ERROR("Could not move arm to DROP pose!");
                return 1;
            }

            // check if we really grabbed the object
            if(!manipulator.moveArmToPose(youbot_proxy::util::Pose::OBSERVE_FAR)) {
                ROS_ERROR("Could not move arm to OBSERVE pose!");
                return 1;
            }
            if(detectObject(objectDetection, maxDetectionDuration, &objectPose)) {
                // there is still an object --> grabbing was not successful
                ROS_WARN("There is still an object in front of the youbot. Retrying grabbing!");

                // try again
                state = TRY_GRASP;
                break;
            }

            // successfull
            nObjects--;
            if(nObjects > 0) {
                state = FIND_NEXT_OBJECT;
            } else {
                state = FINISHED;
            }
            break;

        case ERROR:
        default:
            ROS_ERROR("");
            state = FINISHED;
        }
    }

    // exit cleaner-mode and return youBot to initial pose
    if(!manipulator.returnToInit()) {
      ROS_ERROR("Could not move arm to INIT pose!");
    }

    // deinit
    ros::shutdown();
    return 0;

    /*
    while(nObjects > 0 && ros::ok()) {
        // arm to move pose?
        //TODO

        // find nearest point (--> object)
        waitForValidNearestPoint(nearestPointService, &p);
        ROS_INFO_STREAM("Next Point: x=" << p.x << ", y=" << p.y);

        // move base to the nearest object
        moveToObjectAt(youbot, p, goalPosition);

        // unfold arm
        if(!manipulator.moveArmToPose(youbot_proxy::util::Pose::OBSERVE_FAR)) {
            ROS_ERROR("Could not move arm to OBSERVE pose!");
            return 1;
        }


        // observe object position and orientation in preparation for grasping
        // and grab object
        // ------------------------------------------------------------------------
        // retrieve object position
        int repetitions = 0;
        objectPose = geometry_msgs::Pose();
        while(!detectObject(objectDetection, maxDetectionDuration, &objectPose)) {
            // next point
            // near the scanner and reps < 3?
            // ...
            if(repetitions < 3) {
                ROS_ERROR("");
                return 1;
            }
        }

        // check orientation values
        normalizeOrientation(&objectPose);

        // check position values
        if(checkSelfcollision(objectPose)) {
            ROS_WARN("Grab position would selfcollide");
            // calc correction and back to detectObject-loop
            return 1;
        }
        if(!checkInRange(objectPose)) {
            ROS_WARN("Grab position outside of gripper range");
            // calc correction and back to detectObject-loop
            return 1;
        }

        // grab object
        if(!manipulator.grabObjectAt(objectPose)) {
            ROS_ERROR("Grabbing failed!");
            return 1;
        }

        // drop object on plate
        if(!manipulator.dropObject()) {
            ROS_ERROR("Could not move arm to DROP pose!");
            return 1;
        }

        // check if we really grabbed the object
        if(!manipulator.moveArmToPose(youbot_proxy::util::Pose::OBSERVE_FAR)) {
            ROS_ERROR("Could not move arm to OBSERVE pose!");
            return 1;
        }
        if(detectObject(objectDetection, maxDetectionDuration, &objectPose)) {
            // there is still an object --> grabbing was not successful
            // back to checkSelfcollision()
            ROS_WARN("There is still an object in front of the youbot. Retrying grabbing!");
            return 1;
        }

        nObjects--;
    }


    // exit cleaner-mode and return youBot to initial pose
    if(!manipulator.returnToInit()) {
      ROS_ERROR("Could not move arm to INIT pose!");
    }

    // deinit
    ros::shutdown();
    return 0;
    */
}
