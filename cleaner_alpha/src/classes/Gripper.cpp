#include "cleaner_alpha/Gripper.h"

namespace youbot_proxy {

///////////////////////////////////////////////////////////////////////////////
// const gripper joint names array filler
///////////////////////////////////////////////////////////////////////////////
Gripper::ConstGripperJointNameArrayFiller Gripper::gripperJointArrayFiller = Gripper::ConstGripperJointNameArrayFiller();


///////////////////////////////////////////////////////////////////////////////
// private methods
///////////////////////////////////////////////////////////////////////////////
brics_actuator::JointPositions Gripper::createGripperPositionMsg(double left, double right) const {
    brics_actuator::JointPositions msg;
    brics_actuator::JointValue leftGripperValue;
    brics_actuator::JointValue rightGripperValue;

    leftGripperValue.joint_uri = GRIPPER_JOINT_NAMES[I_FINGER_LEFT];
    leftGripperValue.unit = gripperValueMsgUnit;
    leftGripperValue.value = left;

    rightGripperValue.joint_uri = GRIPPER_JOINT_NAMES[I_FINGER_RIGHT];
    rightGripperValue.unit = gripperValueMsgUnit;
    rightGripperValue.value = right;

    msg.positions.push_back(leftGripperValue);
    msg.positions.push_back(rightGripperValue);
    return msg;
}

void Gripper::sendGripperPositionMsg(const brics_actuator::JointPositions msg) {
    this->gripperPublisher.publish(msg);
}


///////////////////////////////////////////////////////////////////////////////
// public methods
///////////////////////////////////////////////////////////////////////////////
Gripper::Gripper(ros::NodeHandle node, std::string gripperPositionCommand_topic):
    I_FINGER_LEFT(0),
    I_FINGER_RIGHT(1),
    GRIPPER_JOINT_NAMES(gripperJointArrayFiller),
    gripperValueMsgUnit("m"),
    gripperOpen(false)
{
    gripperPublisher = node.advertise<brics_actuator::JointPositions>(gripperPositionCommand_topic, 5);
}

Gripper::~Gripper() {}

void Gripper::openGripper() {
    brics_actuator::JointPositions gripperPositionMsg = this->createGripperPositionMsg(0.0115, 0.0115);
    this->sendGripperPositionMsg(gripperPositionMsg);
    gripperOpen = true;
    ROS_INFO("Gripper opened");
}

void Gripper::closeGripper() {
    brics_actuator::JointPositions gripperPositionMsg = this->createGripperPositionMsg(0.0, 0.0);
    this->sendGripperPositionMsg(gripperPositionMsg);
    gripperOpen = false;
    ROS_INFO("Gripper closed");
}

bool Gripper::isGripperOpen() const {
    return gripperOpen;
}

}
