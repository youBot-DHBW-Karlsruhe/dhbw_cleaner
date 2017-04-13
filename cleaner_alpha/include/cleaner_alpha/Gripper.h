#ifndef GRIPPER_H
#define GRIPPER_H

#include "ros/ros.h"

#include "cleaner_alpha/util/ConstArray.h"

#include "brics_actuator/JointPositions.h"


namespace youbot_proxy {

class Gripper {

    private:
        //----------------------- private classes/types ------------------------
        class ConstGripperJointNameArrayFiller: public util::ConstArrayFiller<std::string, 2> {
        public:
            virtual void fill(std::string* array) {
                addElement(array, "gripper_finger_joint_l");
                addElement(array, "gripper_finger_joint_r");
            }
        };
        static ConstGripperJointNameArrayFiller gripperJointArrayFiller;

        typedef util::ConstArray<std::string, 2> ConstGripperJointNameArray;


        //----------------------- private members -----------------------------
        const std::string gripperValueMsgUnit;

        bool gripperOpen;
        ros::Publisher gripperPublisher;
        // no JointState Subscriber, because there is no state published
        // --> see docs from torque_control package


        //----------------------- private helper methods ----------------------
        brics_actuator::JointPositions createGripperPositionMsg(double left, double right) const;

        void sendGripperPositionMsg(const brics_actuator::JointPositions msg);


    public:
        //----------------------- constants -----------------------------------
        const int I_FINGER_LEFT;
        const int I_FINGER_RIGHT;
        const ConstGripperJointNameArray GRIPPER_JOINT_NAMES;


        //----------------------- public methods ------------------------------
        Gripper(ros::NodeHandle node, std::string gripperPositionCommand_topic = "/arm_1/gripper_controller/position_command");

        virtual ~Gripper();

        void openGripper();

        void closeGripper();

        bool isGripperOpen() const;
};

}

#endif // GRIPPER_H
