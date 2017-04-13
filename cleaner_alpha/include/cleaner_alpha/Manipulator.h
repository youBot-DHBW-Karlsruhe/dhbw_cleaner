#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include "ros/ros.h"
#include "cleaner_alpha/Gripper.h"
#include "cleaner_alpha/TrajectoryGeneratorFactory.h"

#include "cleaner_alpha/util/ConstArray.h"
#include "cleaner_alpha/util/Pose.h"

#include "actionlib/client/simple_action_client.h"

#include "sensor_msgs/JointState.h"
#include "brics_actuator/JointPositions.h"
#include "geometry_msgs/Pose.h"
#include "trajectory_msgs/JointTrajectory.h"

#include "torque_control/torque_trajectoryAction.h"


namespace youbot_proxy {


class Manipulator {

    private:
        class ConstJointNameArrayFiller: public util::ConstArrayFiller<std::string, 5> {
        public:
            virtual void fill(std::string* array) {
                addElement(array, "arm_joint_1");
                addElement(array, "arm_joint_2");
                addElement(array, "arm_joint_3");
                addElement(array, "arm_joint_4");
                addElement(array, "arm_joint_5");
            }
        };
        static ConstJointNameArrayFiller armJointArrayFiller;

        typedef util::ConstArray<std::string, 5> ConstArmJointNameArray;

        ros::Duration timeout;
        sensor_msgs::JointState jointState;

        ros::Publisher gripperPublisher;
        ros::Subscriber armPositionSubscriber;
        actionlib::SimpleActionClient<torque_control::torque_trajectoryAction>* torqueController;

        Gripper gripper;
        TrajectoryGeneratorFactory trajGenFac;

        static const util::Pose createPose(const ConstArmJointNameArray& jointArray);

        brics_actuator::JointPositions extractFirstPointPositions(const trajectory_msgs::JointTrajectory& traj) const;

        void correctGripperOrientationConst(double angle, trajectory_msgs::JointTrajectory& traj) const;

        std::vector<double> quaternionMsgToRPY(const geometry_msgs::Quaternion q) const;

        bool move(const trajectory_msgs::JointTrajectory& traj );

    public:
        // constants
        const double DEFAULT_TIMEOUT;
        const int I_JOINT_1;
        const int I_JOINT_2;
        const int I_JOINT_3;
        const int I_JOINT_4;
        const int I_JOINT_5;
        const ConstArmJointNameArray ARM_JOINT_NAMES;
        const util::Pose POSE;

        Manipulator(ros::NodeHandle& node, const Gripper& pGripper, const TrajectoryGeneratorFactory& tgFactory,
                    std::string jointState_topic = "/joint_states", std::string torqueAction_topic = "/torque_control");

        ~Manipulator();

        void armPositionHandler(const sensor_msgs::JointStateConstPtr& msg);

        sensor_msgs::JointState getJointStates() const;

        void openGripper();

        void closeGripper();

        bool moveArmToPose(util::Pose::POSE_ID poseId);

        bool moveArmToJointPosition(const brics_actuator::JointPositions& targetPosition);


        bool grabObjectAt(const geometry_msgs::Pose& pose);
};

}

#endif // MANIPULATOR_H
