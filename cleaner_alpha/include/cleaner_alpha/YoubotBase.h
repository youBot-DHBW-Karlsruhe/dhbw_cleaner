#ifndef YOUBOT_BASE_H
#define YOUBOT_BASE_H

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

class YoubotBase {
    public:
        // constant members
        static const int DOF = 5;
        static const double TIMEOUT = 15;
        static const double DEFAULT_POINT_SECONDS = 5.0;
        static const double DEFAULT_SPEED = 0.1;

        // constructor
        YoubotBase(ros::NodeHandle& node, std::string base_vel_topic = "/cmd_vel");

        YoubotBase(ros::NodeHandle& node, std::string base_vel_topic, double movementSecondsForEachPoint);

        YoubotBase(ros::NodeHandle& node, std::string base_vel_topic, double movementSecondsForEachPoint, double baseMovementSpeed);

        // destructor
        virtual ~YoubotBase();

        // methods
        /**
         * Moves the base distanceInMeters meter in the specified direction. This function blocks the current thread.
         */
        void move(const Direction *direction, double distanceInMeters);

        /**
         * Move base x meters right and y meters forward. This function blocks the current thread.
         */
        void move(double x, double y);

        /**
         * Turns the base in z (left). Negative values corresponds to the other direction (right).
         */
        void turnRad(double angleInRad);

        /**
         * Turns the base in z (left). Negative values corresponds to the other direction (right).
         */
        void turnDeg(double angleInDeg);

    private:
        // constants
        const double pointSeconds;
        const double speed;
        geometry_msgs::Twist stopMsg;

        // member
        ros::Publisher pubBase;

        // disable copy-constructor and assignment operator
        YoubotBase(const YoubotBase &);
        void operator=(const YoubotBase &);

        // methods
        void initialize(ros::NodeHandle& node, std::string base_vel_topic);
};

}
#endif // YOUBOT_BASE_H
