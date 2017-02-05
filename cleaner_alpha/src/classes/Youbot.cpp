#include "cleaner_alpha/Youbot.h"


namespace youbot_proxy {

///////////////////////////////////////////////////////////////////////////////
// public methods
///////////////////////////////////////////////////////////////////////////////
bool Youbot::initialize(ros::NodeHandle node) {
    // create stop message
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    this->stopMsg = msg;

    // create action client
    this->actionClient = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("arm_1/arm_controller/follow_joint_trajectory", true);
    ROS_INFO_STREAM("Waiting " << this->TIMEOUT << " seconds for action server to start.");
    this->actionClient->waitForServer(ros::Duration(this->TIMEOUT));

    // create publishers
    ros::Publisher armPublisher = node.advertise<brics_actuator::JointPositions>("/arm_1/gripper_controller/position_command", 5);
    ros::Publisher basePublisher = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // check status of members
    if(!armPublisher || !basePublisher) {
        ROS_ERROR("Initialization failed: publishers could not be created");
        return false;
    }
    this->pubArm = armPublisher;
    this->pubBase = basePublisher;

    if(!this->actionClient->isServerConnected()) {
        ROS_ERROR("Initialization failed: simple action client is not connected to a server");
        return false;
    }

    // move the arm to the tower pose and open the gripper
    ROS_INFO("Initializing arm by moving it to the tower position and opening the gripper");
    double pose[this->DOF] = ARM_POSE_TOWER;
    if(!this->moveArmToPose(pose)) {
        ROS_ERROR("Initialization failed: Youbot could not move arm to ARM_POSE_TOWER");
        return false;
    }

    this->openGripper();

    return true;
}

bool Youbot::grab() {
    ROS_INFO("creating demo trajectory points for grab and moving arm");
    double poseGrab[this->DOF] = ARM_POSE_GRAB;
    if(!this->moveArmToPose(poseGrab)) {
        ROS_ERROR("could not move arm to grab pose");
        return false;
    }

    ROS_INFO("closing gripper for grasping");
    this->closeGripper();

    ROS_INFO("returning arm to init position");
    double poseTower[this->DOF] = ARM_POSE_TOWER;
    if(!this->moveArmToPose(poseTower)) {
        ROS_ERROR("could not move arm to tower pose");
        return false;
    }
    return true;
}

bool Youbot::drop() {
    ROS_INFO("creating demo trajectory points for drop and moving arm");
    double poseDrop[this->DOF] = ARM_POSE_DROP;
    if(!this->moveArmToPose(poseDrop)) {
        ROS_ERROR("could not move arm to drop pose");
        return false;
    }

    ROS_INFO("opening gripper for dropping");
    this->openGripper();

    ROS_INFO("returning arm to init position");
    double poseTower[this->DOF] = ARM_POSE_TOWER;
    if(!this->moveArmToPose(poseTower)) {
        ROS_ERROR("could not move arm to tower pose");
        return false;
    }
    return true;
}

bool Youbot::returnToInitPose() {
    ROS_INFO("creating demo trajectory points for initial pose and moving arm");
    double poseInit[this->DOF] = ARM_POSE_INIT;
    if(!this->moveArmToPose(poseInit)) {
        ROS_ERROR("could not move arm to initial pose");
        return false;
    }

    ROS_INFO("closing gripper");
    this->closeGripper();

    return true;
}

void Youbot::openGripper() {
    brics_actuator::JointPositions gripperPositionMsg = this->createGripperPositionMsg(0.0115, 0.0115);
    this->sendGripperPositionMsg(gripperPositionMsg);
    ROS_INFO("Gripper opened");
}

void Youbot::closeGripper() {
    brics_actuator::JointPositions gripperPositionMsg = this->createGripperPositionMsg(0.0, 0.0);
    this->sendGripperPositionMsg(gripperPositionMsg);
    ROS_INFO("Gripper closed");
}

bool Youbot::moveArmToPose(const double pose[DOF]) {
    // create goal and send to action server
    double points[1][this->DOF];
    for (int i = 0; i < this->DOF; i++) {
        points[0][i] = pose[i];
    }
    control_msgs::FollowJointTrajectoryGoal goalMsg = this->createTrajectoryGoal(1, points, this->pointSeconds);
    if(!this->sendTrajectoryGoalAndWaitForResponse("arm_link_0", goalMsg)) {
        ROS_ERROR("No response received from action server for arm trajectory goal");
        return false;
    }
    return true;
}

void Youbot::moveBase(const Direction *direction, double distanceInMeters) {
    geometry_msgs::Twist moveMsg;

    if(speed <= 0) {
        return;
    }

    // diagonal case: calculate distance in x,y and then the time
    double duration = 0;
    if(direction->isDiagonal()) {
        duration = (1/speed) * ((distanceInMeters/2) * std::sqrt(2));
    } else {
        duration = (1/speed) * distanceInMeters;
    }

    switch(direction->type) {
    case LEFT_FORWARD:
        moveMsg.linear.x = speed;
        moveMsg.linear.y = speed;
        break;
    case FORWARD:
        moveMsg.linear.x = speed;
        break;
    case RIGHT_FORWARD:
        moveMsg.linear.x = speed;
        moveMsg.linear.y = -speed;
        break;
    case LEFT:
        moveMsg.linear.y = speed;
        break;
    case RIGHT:
        moveMsg.linear.y = -speed;
        break;
    case LEFT_BACKWARD:
        moveMsg.linear.x = -speed;
        moveMsg.linear.y = speed;
        break;
    case BACKWARD:
        moveMsg.linear.x = -speed;
        break;
    case RIGHT_BACKWARD:
        moveMsg.linear.x = -speed;
        moveMsg.linear.y = -speed;
        break;
    default:
        return;
    }

    this->pubBase.publish(moveMsg);
    ros::Duration(duration).sleep();
    this->pubBase.publish(this->stopMsg);
}

void Youbot::moveBase(double x, double y) {
    geometry_msgs::Twist moveMsg;

    if(speed <= 0) {
        return;
    }

    // calculate different speeds for x and y
    double s, t, vx, vy;
    s  = std::sqrt(std::pow(x, 2) + std::pow(y, 2)); // [s] = [x] = [y] = meter
    t  = s / speed;                                  // [t] = second
    vx = x / t;                                      // [vx] = meter/second
    vy = y / t;                                      // [vy] = meter/second
                                                     // [speed] = meter/second

    moveMsg.linear.x = vx;
    moveMsg.linear.y = vy;
    moveMsg.linear.z =  0;

    this->pubBase.publish(moveMsg);
    ros::Duration(t).sleep();
    this->pubBase.publish(this->stopMsg);
}

void Youbot::turnBaseDeg(double angleInDeg) {
    this->turnBaseRad(DEG_TO_RAD(angleInDeg));
}

void Youbot::turnBaseRad(double angleInRad) {
    //double speed = 0.5;

    if(angleInRad == 0) {
        return;
    }
    if(speed <= 0) {
        return;
    }
    double duration = (1/speed) * std::abs(angleInRad);
    geometry_msgs::Twist turnMsg;
    turnMsg.angular.z = ((angleInRad < 0)? -1 : 1) * speed;

    this->pubBase.publish(turnMsg);
    ros::Duration(duration).sleep();
    this->pubBase.publish(this->stopMsg);
}

///////////////////////////////////////////////////////////////////////////////
// private methods
///////////////////////////////////////////////////////////////////////////////
control_msgs::FollowJointTrajectoryGoal Youbot::createTrajectoryGoal(const int nPoints, const double jointAngles[][DOF], double pointSeconds) {
    control_msgs::FollowJointTrajectoryGoal msg;

    // set values for all points of trajectory
    for (int p = 0; p < nPoints; p++) { // iterate over all points
        trajectory_msgs::JointTrajectoryPoint point;
        for (int i = 0; i < DOF; i++) {
            point.positions.push_back(jointAngles[p][i]);
            point.velocities.push_back(0);
            point.accelerations.push_back(0);
        }
        point.time_from_start = ros::Duration(pointSeconds * (p+1));
        msg.trajectory.points.push_back(point);
    }

    // set joint names
    for (int i = 0; i < 5; i++) {
        std::stringstream jointName;
        jointName << "arm_joint_" << (i + 1);
        msg.trajectory.joint_names.push_back(jointName.str());
    }

    return msg;
}

brics_actuator::JointPositions Youbot::createGripperPositionMsg(double left, double right) {
    brics_actuator::JointPositions msg;
    brics_actuator::JointValue leftGripperValue;
    brics_actuator::JointValue rightGripperValue;

    leftGripperValue.joint_uri = "gripper_finger_joint_l";
    leftGripperValue.unit = "m";
    leftGripperValue.value = left;

    rightGripperValue.joint_uri = "gripper_finger_joint_r";
    rightGripperValue.unit = "m";
    rightGripperValue.value = right;

    msg.positions.push_back(leftGripperValue);
    msg.positions.push_back(rightGripperValue);
    return msg;
}

bool Youbot::sendTrajectoryGoalAndWaitForResponse(const std::string frame, const control_msgs::FollowJointTrajectoryGoal goal) {
    control_msgs::FollowJointTrajectoryGoal trajectoryGoalMsg(goal);

    ROS_INFO("Sending goal to action server and waiting for result...");

    // fill message header and sent it out
    trajectoryGoalMsg.trajectory.header.frame_id = frame;
    trajectoryGoalMsg.trajectory.header.stamp = ros::Time::now();
    this->actionClient->sendGoal(trajectoryGoalMsg);

    // wait for reply that action was completed (or cancel after 10 sec)
    this->actionClient->waitForResult(ros::Duration(this->TIMEOUT));

    if(this->actionClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("...Goal executed successfully");
        return true;
    }
    ROS_WARN("...timeout reached, no result received!");
    return false;
}

void Youbot::sendGripperPositionMsg(const brics_actuator::JointPositions msg) {
    this->pubArm.publish(msg);
}

}
