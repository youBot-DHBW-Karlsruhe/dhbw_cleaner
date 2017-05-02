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

class StateMachine {
private:
    ///////////////////////////////////////////////////////////////////////////////
    // private classes
    ///////////////////////////////////////////////////////////////////////////////
    enum State {
        FIND_NEXT_OBJECT, DRIVE_TO_OBJECT, DETECT_OBJECT, TRY_GRASP, CORRECT_POSITION, CORRECT_GRASP_POSITION, FINISHED, ERROR
    };

    enum Grasp_Result {
        SUCCESSFULL, OBJECT_NOT_REACHABLE, FAILED
    };

    struct ExecutionException: public std::exception {
    private:
        std::string msg;
    public:
        ExecutionException(std::string msg):msg(msg){}
        ~ExecutionException() throw() {}
        const char* what() const throw() {
            return msg.c_str();
        }
    };


    ///////////////////////////////////////////////////////////////////////////////
    // private members
    ///////////////////////////////////////////////////////////////////////////////
    // configuration
    const ros::Duration TIMEOUT;
    const ros::Duration MAX_DETECTION_DURATION; // waiting time until object detection will cancel
    const int N_OBJECTS;
    const geometry_msgs::Point32 GOAL_POSITION;

    // working members
    int correctionAttempts;
    int nObjects;
    geometry_msgs::Point32 p;
    geometry_msgs::Pose objectPose;
    State state;

    // object references
    NearestPointServiceClient nearestPointService;
    ObjectDetectionListener objectDetection;
    youbot_proxy::YoubotBase youbot;
    youbot_proxy::TrajectoryGeneratorFactory tgFactory;
    youbot_proxy::Gripper gripper;
    youbot_proxy::Manipulator manipulator;


    ///////////////////////////////////////////////////////////////////////////////
    // private static methods
    ///////////////////////////////////////////////////////////////////////////////
    static const geometry_msgs::Point32 initGoalPosition() {
        geometry_msgs::Point32 p;
        // assuming that youbot base_link is at (0/0),
        // then target: object at (0/0.39)
        p.x = 0.39;
        p.y = 0;
        return p;
    }

    static bool checkSelfcollision(const geometry_msgs::Pose& objectPose) {
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

    static bool checkInRange(const geometry_msgs::Pose& objectPose) {
        double x = objectPose.position.x;
        double y = objectPose.position.y;
        // relative to frame arm_link_0

        return 0.32 <= std::sqrt(x*x + y*y);
    }

    static bool checkIfPointIsNear(const geometry_msgs::Point32& point) {
        double x = point.x - 0.135; // distance from arm_link_0
        double y = point.y;

        return x < 0.32 && x > 0.23
                && std::abs(y) <=20;
    }

    static bool timedOut(ros::Time startTime, const ros::Duration duration) {
        return ros::Time::now() - startTime >= duration;
    }

    static geometry_msgs::Point32 calcPositionCorrection(const geometry_msgs::Point32& object) {
        geometry_msgs::Point32 movement;
        movement.x = 0;
        movement.y = (object.y > 0)? -0.02 : 0.02;

        // angular movement needed?
        return movement;
    }

    geometry_msgs::Quaternion normalizeOrientation() {
        double roll, pitch, yaw;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(objectPose.orientation, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        ROS_INFO_STREAM("Object Position:\n x=" << objectPose.position.x << ", y=" << objectPose.position.y << ", z=" << objectPose.position.z);
        ROS_INFO_STREAM("Object Orientation:\n Roll=" << roll << ", Pitch=" << pitch << ", Yaw=" << yaw);

        pitch = 0;
        yaw = 0;
        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        objectPose.orientation = q;
        ROS_INFO("Normalized object orientation");
    }


    ///////////////////////////////////////////////////////////////////////////////
    // private state methods
    ///////////////////////////////////////////////////////////////////////////////
    bool findNextObject() {
        // find nearest point (--> object)
        bool found = waitForValidNearestPoint();
        ROS_INFO_STREAM("Next Point: x=" << p.x << ", y=" << p.y);
        return found;
    }

    void driveToObject() {
        double angle, diagMovement;
        // arm to move pose?
        //TODO

        // turn base into direction of the object
        // - -> left
        // + -> right
        angle = std::atan2(p.y, p.x);
        youbot.turnRad(angle);

        // move base straight forward towards the object
        diagMovement = std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2)) - GOAL_POSITION.x;
        youbot.move(diagMovement, GOAL_POSITION.y);
    }

    bool detectObject() {
        ros::Time startWaiting;

        // unfold arm
        if(!manipulator.moveArmToPose(youbot_proxy::util::Pose::OBSERVE_FAR)) {
            ROS_ERROR("Could not move arm to OBSERVE pose!");
            throw ExecutionException("Arm movement failed.");
        }

        // retrieve object position
        ROS_INFO("Waiting for object detection...");
        startWaiting = ros::Time::now();
        while(!objectDetection.foundObject() && ros::ok()) {
            if(timedOut(startWaiting, MAX_DETECTION_DURATION)) {
                ROS_WARN("... no grabbable object found!");
                return false;
            }
        }

        while(!objectDetection.isAvailable() && ros::ok()) {
            if(timedOut(startWaiting, MAX_DETECTION_DURATION)) {
                ROS_WARN("... found object, but could not retrieve aggregated position data");
                return false;
            }
        }
        objectPose = objectDetection.getObjectPosition();
        ROS_INFO("... received object coordinates");
        return true;
    }

    bool correctPosition() {
        // get position of nearest point
        bool found = waitForValidNearestPoint();
        if(!found) {
            throw ExecutionException("No object point found for position correction!");
        }

        // inside bounds?
        if(checkIfPointIsNear(p) && correctionAttempts < 3) {
            // correct position
            geometry_msgs::Point32 movement = calcPositionCorrection(p);
            youbot.move(movement.x, movement.y);
            correctionAttempts++;

            return true;
        } else {
            correctionAttempts = 0;
            return false;
        }
    }

    Grasp_Result tryGrasp() {
        // check position values
        normalizeOrientation();
        ROS_INFO("Checking for self collision");
        if(checkSelfcollision(objectPose)) {
            ROS_WARN("Grab position would selfcollide");
            return OBJECT_NOT_REACHABLE;
        }
        ROS_INFO("Checking for gripper range");
        if(!checkInRange(objectPose)) {
            ROS_WARN("Grab position outside of gripper range");
            return OBJECT_NOT_REACHABLE;
        }

        // grab object
        ROS_INFO("Trying to grab object...");
        if(!manipulator.grabObjectAt(objectPose)) {
            throw ExecutionException("Grabbing failed!");
        }

        // drop object on plate
        if(!manipulator.dropObject()) {
            throw ExecutionException("Could not move arm to DROP pose!");
        }

        // check if we really grabbed the object
        if(detectObject()) {
            ROS_WARN("There is still an object in front of the youbot. Retrying grabbing!");
            // trying again
            return FAILED;
        }

        return SUCCESSFULL;
    }

    void correctGraspPosition() {
        double dx = 0, dy = 0, angle = 0;
        // relative to frame arm_link_0
        double x = objectPose.position.x;
        double y = objectPose.position.y;
        bool selfCollision = checkSelfcollision(objectPose);
        bool outsideRange = !checkInRange(objectPose);

        // calc correction and back to detectObject-loop
        if(outsideRange) {
            angle = std::atan2(x, y);
            youbot.turnRad(angle);

            dx = std::sqrt(x*x + y*y - 0.32*0.32) - 0.04;
            youbot.move(dx, dy);

        } else if(selfCollision) {
            // TODO: adapt or calc dynamically
            dx = -0.04;
            youbot.move(dx, dy);
        }
    }


    ///////////////////////////////////////////////////////////////////////////////
    // private helper methods
    ///////////////////////////////////////////////////////////////////////////////
    bool waitForValidNearestPoint() {
        double tol = 0.03;
        ros::Time startTime = ros::Time::now();
        ros::Rate loopRate(5);
        geometry_msgs::Point32 point = nearestPointService.nearestPoint();

        while(std::abs(point.x) <= tol &&
              std::abs(point.y) <= tol &&
              point.x < 0 &&
              ros::ok()) {
            ROS_WARN_STREAM("Point was nearly (0/0) or negative in x-achsis, tolerance=" << tol << "!!");
            ros::spinOnce();
            loopRate.sleep();
            point = nearestPointService.nearestPoint();

            if(timedOut(startTime, TIMEOUT)) {
                return false;
            }
        }
        p = point;
        return true;
    }

public:

    ///////////////////////////////////////////////////////////////////////////////
    // public methods
    ///////////////////////////////////////////////////////////////////////////////
    StateMachine(ros::NodeHandle n):
        TIMEOUT(30),
        MAX_DETECTION_DURATION(20),
        N_OBJECTS(2),
        GOAL_POSITION(initGoalPosition()),
        correctionAttempts(0),
        nObjects(N_OBJECTS),
        state(FIND_NEXT_OBJECT),

        nearestPointService(n, "nearest_point"),
        objectDetection(n, "object_position_single"),
        youbot(n, "/cmd_vel", youbot_proxy::YoubotBase::DEFAULT_POINT_SECONDS, 0.1),
        tgFactory(n),
        gripper(n, "/arm_1/gripper_controller/position_command"),
        manipulator(n, gripper, tgFactory, "/joint_states", "/torque_control", "/arm_1/arm_controller/follow_joint_trajectory"){}

    ~StateMachine() {}

    void run() {
        try {
            while(state != FINISHED && ros::ok()) {
                switch(state) {
                case FIND_NEXT_OBJECT:
                    if(findNextObject())    state = DRIVE_TO_OBJECT;    // successful
                    else                    state = ERROR;              // failed
                    break;

                case DRIVE_TO_OBJECT:
                    driveToObject();
                    state = DETECT_OBJECT;
                    break;

                case DETECT_OBJECT:
                    if(detectObject())      state = TRY_GRASP;          // successful
                    else                    state = CORRECT_POSITION;   // failed
                    break;

                case CORRECT_POSITION:
                    if(correctPosition())   state = DETECT_OBJECT;      // correction done
                    else                    state = DRIVE_TO_OBJECT;    // no near object, from beginning
                    break;

                case TRY_GRASP: {
                    Grasp_Result result = tryGrasp();
                    if(result == OBJECT_NOT_REACHABLE) {
                        state = CORRECT_GRASP_POSITION; // object outside of gripper range

                    } else if (result == SUCCESSFULL) {
                        nObjects--;
                        if(nObjects > 0)    state = FIND_NEXT_OBJECT; // search the next object
                        else                state = FINISHED;         // all objects picked up

                    } else {
                        state = TRY_GRASP;          // try again

                    }
                    break;
                }

                case CORRECT_GRASP_POSITION:
                    correctGraspPosition();
                    state = DETECT_OBJECT; // or TRY_GRASP again to safe ressources??
                    break;

                case ERROR:
                default:
                    ROS_ERROR("ERROR");
                    state = FINISHED;
                }
            }

            // exit cleaner-mode and return youBot to initial pose
            if(!manipulator.returnToInit()) {
              throw ExecutionException("Could not move arm to INIT pose!");
            }
        } catch(std::exception e) {
            ROS_ERROR_STREAM("There was a runtime error: " << e.what());
        }

        // deinit
        ros::shutdown();
    }
};


//----------------------------------- MAIN -----------------------------------------------
int main(int argc, char** argv)
{
    // init node
    ros::init(argc, argv, "cleaner");
    ros::NodeHandle n;
    StateMachine machine(n);

    ROS_INFO("----- cleaner running ----------------");
    ROS_INFO("--------------------------------------");


    // run this node: main routine
    machine.run();

    return 0;
}
