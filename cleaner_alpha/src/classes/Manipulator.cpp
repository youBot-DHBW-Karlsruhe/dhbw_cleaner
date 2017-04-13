#include "cleaner_alpha/Manipulator.h"

#include "cleaner_alpha/CSTrajectoryGenerator.h"
#include "cleaner_alpha/JSTrajectoryGenerator.h"

#include "tf/transform_datatypes.h"
#include <math.h>


namespace youbot_proxy {


///////////////////////////////////////////////////////////////////////////////
// const gripper joint names array filler
///////////////////////////////////////////////////////////////////////////////
Manipulator::ConstJointNameArrayFiller Manipulator::armJointArrayFiller = Manipulator::ConstJointNameArrayFiller();


///////////////////////////////////////////////////////////////////////////////
// private methods
///////////////////////////////////////////////////////////////////////////////
const util::Pose Manipulator::createPose(const ConstArmJointNameArray& jointArray){
    // create pose object and initialize the poses
    double defaultPose[5] = ARM_POSE_INIT;
    util::Pose pose(jointArray.getArray(), jointArray.size(), defaultPose);
    double poseObserve[5] = ARM_POSE_OBSERVE_FAR;
    pose.addPose(util::Pose::OBSERVE_FAR, poseObserve);
    double poseObserveNear[5] = ARM_POSE_OBSERVE_NEAR;
    pose.addPose(util::Pose::OBSERVE_NEAR, poseObserveNear);
    double poseTower[5] = ARM_POSE_TOWER;
    pose.addPose(util::Pose::TOWER, poseTower);
    double poseDrop[5] = ARM_POSE_DROP;
    pose.addPose(util::Pose::DROP_AT_PLATE, poseDrop);

    return pose;
}

brics_actuator::JointPositions Manipulator::extractFirstPointPositions(const trajectory_msgs::JointTrajectory& traj) const {
    // extract joint positions and names
    const trajectory_msgs::JointTrajectoryPoint point = traj.points.back(); // first point of trajectory
    const std::vector<std::string> joint_names = traj.joint_names;

    // create joint position message
    brics_actuator::JointPositions jointPositions;
    brics_actuator::JointValue val;

    for(int i=0; i<ARM_JOINT_NAMES.size();i++) {
        val.joint_uri = joint_names[i];
        val.unit = "rad";
        val.value = point.positions[i];
        jointPositions.positions.push_back(val);
    }

    return jointPositions;
}

void Manipulator::correctGripperOrientationConst(double angle, trajectory_msgs::JointTrajectory& traj) const {
    ROS_INFO("Correcting gripper orientation ...");

    // find joint_5 index
    double nPoints = traj.points.size();
    double nJoints = ARM_JOINT_NAMES.size();
    int joint5 = 0;

    for(int i=0; i<nJoints; i++) {
        if(traj.joint_names[i] == ARM_JOINT_NAMES[I_JOINT_5]) {
            joint5 = i;
            break;
        }
    }

    // set angle for joint 5 to _angle_ for whole trajectory
    for(int i=0; i<nPoints; i++) {
        trajectory_msgs::JointTrajectoryPoint* point = &traj.points[i];
        if(point->positions.size() == nJoints)     point->positions[joint5] = angle;
        if(point->velocities.size() == nJoints)    point->velocities[joint5] = 0;
        if(point->accelerations.size() == nJoints) point->accelerations[joint5] = 0;
        if(point->effort.size() == nJoints)        point->effort[joint5] = 0;
    }

    ROS_INFO("... finished.");
}

std::vector<double> Manipulator::quaternionMsgToRPY(const geometry_msgs::Quaternion q) const {
    std::vector<double> rpy;
    double roll, pitch, yaw;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(q, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    rpy.push_back(roll);
    rpy.push_back(pitch);
    rpy.push_back(yaw);
    ROS_INFO_STREAM("quaternionMsgToRPY: Roll=" << roll << ", Pitch=" << pitch << ", Yaw=" << yaw);

    return rpy;
}

bool Manipulator::move(const trajectory_msgs::JointTrajectory& traj ) {
    torque_control::torque_trajectoryGoal goal;

    // delay execution a bit to allow callbacks
// TODO: TEST IF REALLY NEEDED ################################################
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    ros::spinOnce();
// ############################################################################

    goal.trajectory = traj;
    torqueController->sendGoal(goal);

    // wait for the action to return
    bool finishedBeforeTimeout = torqueController->waitForResult(ros::Duration(20.0));
    if (finishedBeforeTimeout) {
        ROS_INFO("Action finished: %s", torqueController->getState().toString().c_str());
        return true;
    } else {
        ROS_INFO("Action did not finish before the time out.");
        return false;
    }
}


///////////////////////////////////////////////////////////////////////////////
// public methods
///////////////////////////////////////////////////////////////////////////////
Manipulator::Manipulator(ros::NodeHandle& node, const Gripper& pGripper, const TrajectoryGeneratorFactory& tgFactory,
                         std::string jointState_topic, std::string torqueAction_topic):
    DEFAULT_TIMEOUT(20),
    I_JOINT_1(0),
    I_JOINT_2(1),
    I_JOINT_3(2),
    I_JOINT_4(3),
    I_JOINT_5(4),
    ARM_JOINT_NAMES(armJointArrayFiller),
    timeout(ros::Duration(DEFAULT_TIMEOUT)),
    gripper(pGripper),
    trajGenFac(tgFactory),
    POSE(createPose(ARM_JOINT_NAMES))
{
    armPositionSubscriber = node.subscribe<sensor_msgs::JointState>(jointState_topic, 1, &Manipulator::armPositionHandler, this);

    torqueController = new actionlib::SimpleActionClient<torque_control::torque_trajectoryAction>(torqueAction_topic, true);

    ROS_INFO_STREAM("Waiting " << timeout << " seconds for action server to start");
    torqueController->waitForServer(timeout);
    if(!torqueController->isServerConnected()) {
        ROS_ERROR("Initialization failed: torque controller is not available");
        exit(1);
// throw error instead of shutting down the process ###########################
    }
}

Manipulator::~Manipulator() {
    delete torqueController;
}

void Manipulator::armPositionHandler(const sensor_msgs::JointStateConstPtr& msg) {
    sensor_msgs::JointState state;

    // only save arm joint values
    for(int i=0; i<ARM_JOINT_NAMES.size(); i++) {
        if(ARM_JOINT_NAMES[i] != msg->name[i]) {
            return;
        }
        state.name.push_back(ARM_JOINT_NAMES[i]);
        state.position.push_back(msg->position[i]);
        state.velocity.push_back(msg->velocity[i]);
        state.effort.push_back(msg->effort[i]);
    }

    state.header = msg->header;
    jointState = state;
}

sensor_msgs::JointState Manipulator::getJointStates() const {
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    ros::spinOnce();
    return jointState;
}

void Manipulator::openGripper() {
    gripper.openGripper();
}

void Manipulator::closeGripper() {
    gripper.closeGripper();
}

bool Manipulator::moveArmToPose(util::Pose::POSE_ID poseId) {
// ############################################################################
// TODO: test
    return moveArmToJointPosition(POSE.jointPositions(poseId));
// ############################################################################
}

bool Manipulator::moveArmToJointPosition(const brics_actuator::JointPositions& targetPosition) {
    ROS_INFO("Generating trajectory to joint position (js2js)");

    // extract current joint positions
    sensor_msgs::JointState jointState = this->getJointStates();
    brics_actuator::JointPositions currentJointPositions = youbot_proxy::CSTrajectoryGenerator::jointStateToJointPositions(jointState);

    // create trajectory from current joint position to target position
    // js2js
    trajectory_msgs::JointTrajectory js2jsTraj;
    youbot_proxy::JSTrajectoryGenerator trajectoryGenJs = this->trajGenFac.getJSTrajectoryGenerator(currentJointPositions);
    if(!trajectoryGenJs.addPosition(targetPosition)) {
        ROS_ERROR("Could not create trajectory to target position");
        return false;
    }
    js2jsTraj = trajectoryGenJs.get();

    // move arm
    ROS_INFO("Moving arm to target position (js2js)");
    if(!this->move(js2jsTraj)) {
        ROS_ERROR("Could not move arm to target position. Execution of js2js trajectory failed.");
        return false;
    }
    return true;
}

bool Manipulator::grabObjectAt(const geometry_msgs::Pose& pose) {
    // configuration
    double dGrabLine = 0.07;
    double gripperExt = 0; // TODO: test
    double joint_5_min = 0.1221730476;
    double joint_1_Aty0 = 2.9499152248919236;

    // extract rpy
    std::vector<double> rpy = quaternionMsgToRPY(pose.orientation);

    ROS_INFO_STREAM("Roll=" << rpy[0] << ", Pitch=" << rpy[1] << ", Yaw=" << rpy[2]);

    // check roll, pitch and yaw
    if(rpy[0] < -M_PI || rpy[0] > M_PI) {
        ROS_ERROR("Roll must not be within ]-PI, PI[");
        return false;
    }
    if(rpy[1] != 0) {
        ROS_ERROR("Pitch must be 0");
        return false;
    }
    if(rpy[2] != 0) {
        ROS_ERROR("Yaw must be 0");
        return false;
    }

    ///////////////////////////////////////////////////////////////////////////
    // setup phase
    ///////////////////////////////////////////////////////////////////////////
    ROS_INFO("Generating linear trajectory (cs2cs)");
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0, M_PI_2, 0.0);

    // create IK start position
    geometry_msgs::Pose armStartPosition;
    armStartPosition.orientation = q;
    armStartPosition.position.x = pose.position.x;
    armStartPosition.position.y = pose.position.y;
    armStartPosition.position.z = pose.position.z + gripperExt + dGrabLine;


    // create IK goal position
    geometry_msgs::Pose armGoalPosition;
    armGoalPosition.orientation = q;
    armGoalPosition.position.x = pose.position.x;
    armGoalPosition.position.y = pose.position.y;
    armGoalPosition.position.z = pose.position.z + gripperExt;


    // create linear trajectory
    // cs2cs
    trajectory_msgs::JointTrajectory cs2csTraj;
    youbot_proxy::CSTrajectoryGenerator trajectoryGenCS = this->trajGenFac.getCSTrajectoryGenerator(armStartPosition);
    if(!trajectoryGenCS.addPose(armGoalPosition)) {
        ROS_ERROR("Could not create linear grab trajectory");
        return false;
    }
    cs2csTraj = trajectoryGenCS.get();


    ROS_INFO("Generating trajectory to first position (js2js)");
    // extract current joint positions
    sensor_msgs::JointState jointState = this->getJointStates();
    brics_actuator::JointPositions currentJointPositions = youbot_proxy::CSTrajectoryGenerator::jointStateToJointPositions(jointState);

    // extract first point
    brics_actuator::JointPositions firstJointPositions = this->extractFirstPointPositions(cs2csTraj);

    ROS_INFO("Calculating new gripper orientation");
    double j1_current;
    for(int i=0; i<firstJointPositions.positions.size(); i++) {
        if(firstJointPositions.positions[i].joint_uri == "arm_joint_1") {
            j1_current = firstJointPositions.positions[i].value;
            break;
        }
    }
    // calculate gripper joint angle
    double j1_correction = j1_current - joint_1_Aty0;
    double joint_limit_correction = - M_PI_2*joint_5_min; //i am a motherfucking genius
    double angle = rpy[0] + M_PI + joint_limit_correction + j1_correction; // gripper should be directed forward if roll==0, therefore add PI

    // correct gripper orientation
    for(int i=0; i<firstJointPositions.positions.size(); i++) {
        if(firstJointPositions.positions[i].joint_uri == "arm_joint_5") {
            firstJointPositions.positions[i].value = angle;
            break;
        }
    }

    // create trajectory from current joint position to first position of linear trajectory
    // js2js
    trajectory_msgs::JointTrajectory js2jsTraj;
    youbot_proxy::JSTrajectoryGenerator trajectoryGenJs = this->trajGenFac.getJSTrajectoryGenerator(currentJointPositions);
    if(!trajectoryGenJs.addPosition(firstJointPositions)) {
        ROS_ERROR("Could not create trajectory to first position");
        return false;
    }
    js2jsTraj = trajectoryGenJs.get();

    // correct orientation of gripper
    ROS_INFO("Correcting gripper orientation");
    this->correctGripperOrientationConst(angle, cs2csTraj);


    ///////////////////////////////////////////////////////////////////////////
    // actual movement phase
    ///////////////////////////////////////////////////////////////////////////
    // open gripper
    this->openGripper();

    // move arm to start pose
    ROS_INFO("Moving arm to first position (js2js)");
    if(!this->move(js2jsTraj)) {
        ROS_ERROR("Could not move arm to start position. Execution of js2js trajectory failed.");
        return false;
    }

    // move arm to grab position
    ROS_INFO("Moving arm to grab position (cs2cs)");
    if(!this->move(cs2csTraj)) {
        ROS_ERROR("Could not move arm to grab position. Execution of cs2cs trajectory failed.");
        return false;
    }

    // close gripper
    this->closeGripper();
    ros::spinOnce();
    ros::Duration(2.0).sleep();

    // move arm back to start position
    ROS_INFO("Moving arm back to start position (cs2cs)");
    TrajectoryGenerator::reverseTrajectory(cs2csTraj);
    if(!this->move(cs2csTraj)) {
        ROS_ERROR("Could not move arm to start position. Execution of cs2cs trajectory failed.");
        return false;
    }

    // move arm to intermediate position
    ROS_INFO("Moving arm to inermediate position");
    brics_actuator::JointPositions interPos = currentJointPositions; // in this case it's the init position
    this->moveArmToJointPosition(interPos);

    // finished
    return true;
}

bool Manipulator::returnToInit() {
    closeGripper();
    return moveArmToPose(util::Pose::INIT);
}

}
