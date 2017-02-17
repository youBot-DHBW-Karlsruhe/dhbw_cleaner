#ifndef CLEANER_ALPHA_YOUBOT_H
#define CLEANER_ALPHA_YOUBOT_H

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "brics_actuator/JointPositions.h"
#include "geometry_msgs/Twist.h"

namespace youbot_proxy
{

#define ARM_POSE_INIT   {0.00004027682889217683, -0.0, -0.00010995574287564276, 0.0008185840012874813, 0.0023451325442290006}
#define ARM_POSE_TOWER  {2.994239875087764, 1.1520884828390492, -2.700355965393107, 2.000221069091924, 2.9442475376037303}
#define ARM_POSE_GRAB   {2.953187717239413, 2.4635926544333344, -1.7269394542799927, 2.8039599388965972, 2.933296211100019}
#define ARM_POSE_DROP   {2.94689446272501, 0.08719933455156284, -2.822768123140233, 0.053185836191759595, 5.855950830183301}

// degree to radiant conversion
#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)
#define RAD_TO_DEG(x) ((x) * 180.0 / M_PI)


enum Type {
    LEFT_FORWARD = 1, FORWARD, RIGHT_FORWARD,
    LEFT, STOP, RIGHT,
    LEFT_BACKWARD, BACKWARD, RIGHT_BACKWARD
};

class Direction {
    public:
        const Type type;

        Direction(Type pType): type(pType){}
        ~Direction(){}

        bool isDiagonal() const;
};

class Youbot {
    public:
        // constant members
        static const int DOF = 5;
        static const double TIMEOUT = 15;
        static const double DEFAULT_POINT_SECONDS = 5.0;
        static const double DEFAULT_SPEED = 0.1;

        // constructor
        Youbot():
            pointSeconds(DEFAULT_POINT_SECONDS),
            speed(DEFAULT_SPEED){}

        Youbot(double movementSecondsForEachPoint):
            pointSeconds(std::abs(movementSecondsForEachPoint)),
            speed(DEFAULT_SPEED){}

        Youbot(double movementSecondsForEachPoint, double baseMovementSpeed):
            pointSeconds(std::abs(movementSecondsForEachPoint)),
            speed(std::abs(baseMovementSpeed)){}

        // destructor
        ~Youbot() {
            delete actionClient;
        }

        // methods
        /**
         * Initializes the youBot.
         */
        bool initialize(ros::NodeHandle node);

        /**
         * Performs an object grab in front of the youBot.
         */
        bool grab();

        /**
         * Drops a grabbed object on the metal plate of the youBot.
         */
        bool drop();

        /**
         * Returns arm and gripper to the initial pose of the youbot driver.
         */
        bool returnToInitPose();

        /**
         * Open gripper -> drop an object.
         */
        void openGripper();

        /**
         * Close gripper -> grab an object.
         */
        void closeGripper();

        /**
         * Moves youBot arm to the specified pose. The pose is in joint space and depends on the DOF member.
         */
        bool moveArmToPose(const double pose[DOF]);

        /**
         * Moves the base distanceInMeters meter in the specified direction. This function blocks the current thread.
         */
        void moveBase(const Direction *direction, double distanceInMeters);

        /**
         * Move base x meters right and y meters forward. This function blocks the current thread.
         */
        void moveBase(double x, double y);

        /**
         * Turns the base in z (left). Negative values corresponds to the other direction (right).
         */
        void turnBaseRad(double angleInRad);

        /**
         * Turns the base in z (left). Negative values corresponds to the other direction (right).
         */
        void turnBaseDeg(double angleInDeg);

    private:
        // constants
        const double pointSeconds;
        const double speed;
        geometry_msgs::Twist stopMsg;

        // member
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* actionClient;
        ros::Publisher pubArm;
        ros::Publisher pubBase;

        // disable copy-constructor and assignment operator
        Youbot(const Youbot &);
        void operator=(const Youbot &);

        // methods
        /**
         * Creates a trajectory message compliant to the youBot follow_joint_trajectory topic.
         */
        control_msgs::FollowJointTrajectoryGoal createTrajectoryGoal(int nPoints, const double jointAngles[][DOF], double pointSeconds);

        /**
         * Creates a message compliant to the youBot gripper_controller/position_command topic.
         */
        brics_actuator::JointPositions createGripperPositionMsg(double left, double right);

        /**
         * Sends a trajectory message to the youBot arm.
         */
        bool sendTrajectoryGoalAndWaitForResponse(const std::string frame, const control_msgs::FollowJointTrajectoryGoal goal);

        /**
         * Sends a position message to the gripper on arm_1.
         */
        void sendGripperPositionMsg(const brics_actuator::JointPositions msg);

};

}
#endif // CLEANER_ALPHA_YOUBOT_HH
