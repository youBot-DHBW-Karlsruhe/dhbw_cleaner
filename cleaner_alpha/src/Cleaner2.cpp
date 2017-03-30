#include "ros/ros.h"

#include "torque_control/torque_trajectoryAction.h"
#include "actionlib/client/simple_action_client.h"
#include "trajectory_generator/CStoCS.h"
#include "trajectory_generator/JStoCS.h"
#include "trajectory_generator/JStoJS.h"

#include "control_msgs/FollowJointTrajectoryAction.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "brics_actuator/JointPositions.h"
#include "sensor_msgs/JointState.h"

#include "Eigen/Dense"
#include "tf/transform_datatypes.h"

#include <math.h>

namespace youbot_proxy {

class Trajectory {
private:
    ros::ServiceClient cs2csGenerator;
    ros::ServiceClient js2csGenerator;

    trajectory_generator::CStoCS cs2cs;
    trajectory_generator::JStoCS js2cs;
    trajectory_msgs::JointTrajectory t;

    double max_acc;
    double max_vel;
    int keyPointCounter; // beginning at 1 = start position
    bool isJs2Cs;
    bool valid;

    void resetCS2CS(const geometry_msgs::Pose start, double vel) {
        isJs2Cs = false;
        ++keyPointCounter;

        cs2cs.request.start_pos = start;
        cs2cs.request.start_vel = checkedVel(vel);
        cs2cs.request.max_acc = max_acc;
        cs2cs.request.max_vel = max_vel;
    }

    template <typename T> // T is either   JStoCS or CStoCS   of namespace trajectory_generator
    bool callGenerator(ros::ServiceClient& generator, T& msg) {     // !! no const
        BOOST_STATIC_ASSERT(((boost::is_base_of<trajectory_generator::JStoCS, T>::value) || (boost::is_base_of<trajectory_generator::CStoCS, T>::value))); //Yes, the double parentheses are needed, otherwise the comma will be seen as macro argument separator

        bool feasible = false;
        if(generator.call(msg)) {
            feasible = msg.response.feasible;
            if(feasible) {
                // extract trajectory points and append them to member 't'
                extractAndStoreTrajectory(msg.response.trajectory);
                return true;
            } else {
                ROS_ERROR("Could not add point to trajectory. Trajectory not feasible!");
                return false;
            }
        } else {
            ROS_ERROR("Could not add point to trajectory. Call to trajectory generator failed!");
            return false;
        }
    }

    void extractAndStoreTrajectory(trajectory_msgs::JointTrajectory trajectory) {
        trajectory_msgs::JointTrajectoryPoint point;

        while (!trajectory.points.empty()) {
            point = trajectory.points.back();
            trajectory.points.pop_back();
            t.points.insert(t.points.begin(), point);
        }
        t.joint_names = trajectory.joint_names;
    }

    double checkedVel(double vel) {
        if(vel > 0.199999) {
            return 0.199999;
        } else if(vel <= .0) {
            return 0.000001;
        } else {
            return vel;
        }
    }

    double checkedAcc(double acc) {
        if(acc > 0.999999) {
            return 0.999999;
        } else if (acc <= .0) {
            return 0.000001;
        } else {
            return acc;
        }
    }


public:
    static const double DEFAULT_MAX_VEL = 0.05;
    static const double DEFAULT_MAX_ACC = 0.25;

    static const brics_actuator::JointPositions jointStateToJointPositions(const sensor_msgs::JointState jointState) {
        brics_actuator::JointPositions positions;
        brics_actuator::JointValue val;
        val.unit = "rad";

        for(int i=0; i<jointState.name.size(); i++) {
            val.joint_uri = jointState.name[i];
            val.value = jointState.position[i];
            val.timeStamp = jointState.header.stamp;
            positions.positions.push_back(val);
        }

        return positions;
    }

    static const geometry_msgs::Pose createPose(double x, double y, double z, const geometry_msgs::Quaternion gripperOrientation) {
        geometry_msgs::Pose kp;
        kp.position.x = x;
        kp.position.y = y;
        kp.position.z = z;
        kp.orientation = gripperOrientation;
        return kp;
    }

    Trajectory(const ros::ServiceClient& cs2csGen, const ros::ServiceClient& js2csGen, const brics_actuator::JointPositions startPosition) {
        cs2csGenerator = cs2csGen;
        js2csGenerator = js2csGen;
        max_acc = DEFAULT_MAX_ACC;
        max_vel = DEFAULT_MAX_VEL;

        js2cs.request.start_pos = startPosition;
        js2cs.request.start_vel = checkedVel(0.0);
        js2cs.request.max_acc = max_acc;
        js2cs.request.max_vel = max_vel;

        keyPointCounter = 1;
        isJs2Cs = true;
        valid = false;
    }

    Trajectory(const ros::ServiceClient& cs2csGen, const geometry_msgs::Pose startPose) {
        cs2csGenerator = cs2csGen;
        max_acc = DEFAULT_MAX_ACC;
        max_vel = DEFAULT_MAX_VEL;

        resetCS2CS(startPose, 0.0);

        keyPointCounter = 1;
        isJs2Cs = false;
        valid = false;
    }

    void setMaxVel(double vel) {
        max_vel = checkedVel(vel);
    }

    void setMaxAcc(double acc) {
        max_acc = checkedAcc(acc);
    }

    bool createAndAddPose(double x, double y, double z, const geometry_msgs::Quaternion gripperOrientation, double vel) {
        geometry_msgs::Pose p = Trajectory::createPose(x, y, z, gripperOrientation);
        return addPose(p, vel);
    }

    bool addPose(const geometry_msgs::Pose p, double vel = 0.) {
        if(isJs2Cs) {
            // use js2cs
            js2cs.request.end_pos = p;
            js2cs.request.end_vel = checkedVel(vel);

            if(callGenerator(js2csGenerator, js2cs)) {
                // reset cs2cs object to start over at this pose
                resetCS2CS(p, vel);
                valid = true;
                return true;
            }
            return false;

        } else {
            // use cs2cs
            cs2cs.request.end_pos = p;
            cs2cs.request.end_vel = checkedVel(vel);

            if(callGenerator(cs2csGenerator, cs2cs)) {
                // reset cs2cs object to start over at this pose
                resetCS2CS(p, vel);
                valid = true;
                return true;
            }
            return false;
        }
    }

    trajectory_msgs::JointTrajectory get(bool stayAvailable = false) {
        trajectory_msgs::JointTrajectory temp = t;
        if(!isValid()) {
            ROS_ERROR("Trajectory not valid!");
            return temp;
        }

        ROS_INFO_STREAM("Returning trajectory from " << keyPointCount() << " key points");
        if(!stayAvailable) {
            t.joint_names.clear();
            t.points.clear();
            keyPointCounter = 1;
        }

        return temp;
    }

    int keyPointCount() {
        return keyPointCounter;
    }

    bool isValid() {
        return valid;
    }
};


class Manipulator {

    private:

        ros::Duration timeout;
        sensor_msgs::JointState jointState;

        ros::ServiceClient cs2csGenerator;
        ros::ServiceClient js2csGenerator;
        ros::ServiceClient js2jsGenerator;

        ros::Publisher armPublisher;
        ros::Subscriber armPositionSubscriber;

        actionlib::SimpleActionClient<torque_control::torque_trajectoryAction>* torqueController;

    public:
        // constants
        static const int N_JOINTS = 5;
        static const double DEFAULT_TIMEOUT = 20;
        static const std::string JOINT_NAMES[N_JOINTS];

        Manipulator(ros::NodeHandle node) {
            timeout = ros::Duration(DEFAULT_TIMEOUT);

            cs2csGenerator = node.serviceClient<trajectory_generator::CStoCS>("From_CS_to_CS");
            js2csGenerator = node.serviceClient<trajectory_generator::JStoCS>("From_JS_to_CS");
            js2jsGenerator = node.serviceClient<trajectory_generator::JStoJS>("From_JS_to_JS");

            armPublisher = node.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command", 1);
            armPositionSubscriber = node.subscribe<sensor_msgs::JointState>("/joint_states", 1, &Manipulator::armPositionHandler, this);

            torqueController = new actionlib::SimpleActionClient<torque_control::torque_trajectoryAction>("torque_control", true);
            ROS_INFO_STREAM("Waiting " << timeout << " seconds for action server to start");
            torqueController->waitForServer(timeout);
            if(!torqueController->isServerConnected()) {
                ROS_ERROR("Initialization failed: torque controller is not available");
                EXIT_FAILURE;
            }
        }

        ~Manipulator() {
            delete torqueController;
        }

        Trajectory newCS2CSTrajectory(double x, double y, double z, const geometry_msgs::Quaternion gripperOrientation) const {
            return Trajectory(cs2csGenerator, Trajectory::createPose(x, y, z, gripperOrientation));
        }

        Trajectory newCS2CSTrajectory(const geometry_msgs::Pose start) const {
            return Trajectory(cs2csGenerator, start);
        }

        Trajectory newSmartTrajectory() const {
            sensor_msgs::JointState jointState = getJointStates();
            brics_actuator::JointPositions jointPositions = Trajectory::jointStateToJointPositions(jointState);
            std::stringstream ss;
            for(int i=0; i< jointPositions.positions.size(); i++) {
                ss << jointPositions.positions[i] << ", ";
            }
            ROS_INFO_STREAM("Current position: " << ss.str());
            return Trajectory(cs2csGenerator, js2csGenerator, jointPositions);
        }

        void armPositionHandler(const sensor_msgs::JointStateConstPtr& msg) {
            sensor_msgs::JointState state;

            // only save arm joint values
            for(int i=0; i<N_JOINTS; i++) {
                if(JOINT_NAMES[i] != msg->name[i]) {
                    return;
                }
                state.name.push_back(JOINT_NAMES[i]);
                state.position.push_back(msg->position[i]);
                state.velocity.push_back(msg->velocity[i]);
                state.effort.push_back(msg->effort[i]);
            }

            state.header = msg->header;
            jointState = state;
        }

        sensor_msgs::JointState getJointStates() const {
            ros::spinOnce();
            ros::Duration(0.5).sleep();
            ros::spinOnce();
            return jointState;
        }

        bool move(Trajectory trajectory) {
            torque_control::torque_trajectoryGoal goal;

            // check trajectory
            if(!trajectory.isValid()) {
                ROS_INFO("Trajectory is not valid!");
                return false;
            }

            // delay execution a bit to allow callbacks
            ros::spinOnce();
            ros::Duration(0.5).sleep();
            ros::spinOnce();

            goal.trajectory = trajectory.get();
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

        bool moveTEST(const trajectory_msgs::JointTrajectory& traj ) {
            torque_control::torque_trajectoryGoal goal;

            // delay execution a bit to allow callbacks
            ros::spinOnce();
            ros::Duration(0.5).sleep();
            ros::spinOnce();

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

        bool moveTo(double x, double y, double z, const geometry_msgs::Quaternion gripperOrientation) {
            return moveTo(Trajectory::createPose(x, y, z, gripperOrientation));
        }

        bool moveTo(geometry_msgs::Pose pose) {
            torque_control::torque_trajectoryGoal goal;

            // use actual arm position as start pose
            Trajectory trajectory = newSmartTrajectory();
            if(!trajectory.addPose(pose, 0.0)) {
                ROS_ERROR("Unable to create trajectory for pose");
                return false;
            }

            // check trajectory
            if(!trajectory.isValid()) {
                ROS_INFO("Trajectory is not valid!");
                return false;
            }

            // delay execution a bit to allow callbacks
            ros::spinOnce();
            ros::Duration(0.5).sleep();
            ros::spinOnce();

            goal.trajectory = trajectory.get();
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

        // FOR TESTING ONLY!!
        // FIXME: delete
        void publish(const brics_actuator::JointPositions& msg) {
            armPublisher.publish(msg);
        }

};

const std::string Manipulator::JOINT_NAMES[] = {"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"};

}

//-------------------------------- FUNCTIONS ---------------------------------------------
bool moveToStartPose(ros::NodeHandle node, youbot_proxy::Manipulator& m, const brics_actuator::JointPositions& msg) {
    ros::ServiceClient js2jsGenerator = node.serviceClient<trajectory_generator::JStoJS>("From_JS_to_JS");
    trajectory_generator::JStoJS js2js;

    // goal joint position
    brics_actuator::JointPositions jointPositionsTarget = msg;

    // current joint position
    sensor_msgs::JointState jointState = m.getJointStates();
    brics_actuator::JointPositions jointPositionsStart = youbot_proxy::Trajectory::jointStateToJointPositions(jointState);

    // print current joint positions
    std::stringstream ss1, ss2;
    for(int i=0; i< jointPositionsStart.positions.size(); i++) {
        ss1 << jointPositionsStart.positions[i].joint_uri << ": " <<
              jointPositionsStart.positions[i].value << " " <<
              jointPositionsStart.positions[i].unit << "\n";
        ss2 << jointPositionsTarget.positions[i].joint_uri << ": " <<
              jointPositionsTarget.positions[i].value << " " <<
              jointPositionsTarget.positions[i].unit << "\n";
    }
    ROS_INFO_STREAM("Current position: " << ss1.str());
    ROS_INFO_STREAM("Target position: " << ss2.str());

    // create velocity msgs
    brics_actuator::JointVelocities zeroVelocity;
    brics_actuator::JointValue zero;
    zero.unit = "s^-1 rad";
    zero.value = 0.00;
    for(int j=0; j<5; j++) {
        // create zero velocity msg
        std::stringstream st;
        st << "arm_joint_" << (j+1);
        zero.joint_uri = st.str().c_str();
        zeroVelocity.velocities.push_back(zero);
    }

    // create service message
    js2js.request.start_pos = jointPositionsStart;
    js2js.request.start_vel = zeroVelocity;
    js2js.request.end_pos = jointPositionsTarget;
    js2js.request.end_vel = zeroVelocity;

    js2js.request.max_vel = youbot_proxy::Trajectory::DEFAULT_MAX_VEL;
    js2js.request.max_acc = youbot_proxy::Trajectory::DEFAULT_MAX_ACC;

    // call generator
    bool feasible = false;
    if(js2jsGenerator.call(js2js)) {
        feasible = js2js.response.feasible;
        if(feasible) {
            // extract trajectory points and execute
            if(!m.moveTEST(js2js.response.trajectory)) {
                ROS_ERROR("JS2JS Trajectory executation failed!");
                return false;
            }
            return true;
        } else {
            ROS_ERROR("Could not add point to trajectory. Trajectory not feasible!");
            return false;
        }
    } else {
        ROS_ERROR("Could not add point to trajectory. Call to trajectory generator failed!");
        return false;
    }
}

brics_actuator::JointPositions extractFirstPointPositions(const trajectory_msgs::JointTrajectory& traj) {
    const int n = 5; // TODO: use joint array
    // extract joint positions
    const trajectory_msgs::JointTrajectoryPoint point = traj.points.back(); // first point of trajectory
    double firstPt[n];
    for(int i=0; i<n; i++) {
        firstPt[i] = point.positions.back();
    }

    // create joint position message
    brics_actuator::JointPositions jointPositions;
    brics_actuator::JointValue val;
    val.unit = "rad";

    // TODO: optimize with JOINT_NAMES-array
    for(int i=0; i<n;i++) {
        std::stringstream ss;
        ss.str("");
        ss << "arm_joint_" << (i+1);
        val.joint_uri = ss.str();
        val.value = firstPt[i];
        jointPositions.positions.push_back(val);
    }

    return jointPositions;
}

void correctGripperOrientation(double yaw, trajectory_msgs::JointTrajectory& traj) {
    const int JOINT5 = 4;
    // positions
    double n = traj.points.size();
    double origPos = traj.points[0].positions[JOINT5];
    double goalPos = yaw;
    double incPos = (goalPos - origPos) / n;
    // velocities
    double origVel = traj.points[0].velocities[JOINT5];
    double goalVel = traj.points[n].velocities[JOINT5];
    double incVel = (goalVel - origVel) / n;
    // accelerations
    double origAcc = traj.points[0].accelerations[JOINT5];
    double goalAcc = traj.points[n].accelerations[JOINT5];
    double incAcc = (goalAcc - origAcc) / n;

    // effort?

    // interpolate positions
    for(int i=0; i<n-1; i++) {
        trajectory_msgs::JointTrajectoryPoint* point = &traj.points[i];
        point->positions[JOINT5] = origPos + i * incPos;
        point->velocities[JOINT5] = origVel + i * incVel;
        point->accelerations[JOINT5] = origAcc + i * incAcc;
        // effort?
    }

    // end state:
    trajectory_msgs::JointTrajectoryPoint* lastPoint = &traj.points[n-1];
    lastPoint->positions[JOINT5] = goalPos;
    lastPoint->velocities[JOINT5] = goalVel;
    lastPoint->accelerations[JOINT5] = goalAcc;
    // effort?
}

bool grabObjectAt(const geometry_msgs::Pose& pose, youbot_proxy::Manipulator& m) {
    // configuration
    double dGrabLine = 0.1;
    double gripperExt = 0; // TODO: test

    // extract rpy
    double roll, pitch, yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // check roll and pitch
    if(roll != 0) {
        ROS_ERROR("Roll must be 0");
        return false;
    }
    if(std::abs(pitch - M_PI_2) > 0.1) {
        ROS_ERROR("Pitch must be around PI/2");
        return false;
    }

    ///////////////////////////////////////////////////////////////////////////
    // setup phase
    ///////////////////////////////////////////////////////////////////////////

    // create IK goal position
    geometry_msgs::Pose armGoalPosition;
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0, pitch, 0.0);
    armGoalPosition.orientation = q;
    armGoalPosition.position.x = pose.position.x;
    armGoalPosition.position.y = pose.position.y;
    armGoalPosition.position.z = pose.position.z + gripperExt;

    // create IK start position
    geometry_msgs::Pose armStartPosition;
    armStartPosition.orientation = q;
    armStartPosition.position.x = pose.position.x;
    armStartPosition.position.y = pose.position.y;
    armStartPosition.position.z = pose.position.z + gripperExt + dGrabLine;

    // create linear trajectory
    // cs2cs
    trajectory_msgs::JointTrajectory cs2csTraj;
    youbot_proxy::Trajectory trajectoryGen = m.newCS2CSTrajectory(armStartPosition);
    if(!trajectoryGen.addPose(armGoalPosition)) {
        ROS_ERROR("Could not create linear grab trajectory");
        return false;
    }
    cs2csTraj = trajectoryGen.get();


    // extract first point
    brics_actuator::JointPositions firstJointPositions = extractFirstPointPositions(cs2csTraj);

    // extract current joint positions
    sensor_msgs::JointState jointState = m.getJointStates();
    brics_actuator::JointPositions currentJointPositions = youbot_proxy::Trajectory::jointStateToJointPositions(jointState);

    // create trajectory from current joint position to first position of linear trajectory
    // js2js
    trajectory_msgs::JointTrajectory js2jsTraj;
    // TODO:

    // correct orientation of gripper with yaw
    correctGripperOrientation(yaw, cs2csTraj);


    ///////////////////////////////////////////////////////////////////////////
    // actual movement phase
    ///////////////////////////////////////////////////////////////////////////

    // move arm to start pose
    if(!m.moveTEST(js2jsTraj)) {
        ROS_ERROR("Could not move arm to start position. Execution of js2js trajectory failed.");
        return false;
    }

    // move arm to grab position
    if(!m.moveTEST(js2jsTraj)) {
        ROS_ERROR("Could not move arm to start position. Execution of js2js trajectory failed.");
        return false;
    }

    // close gripper


    // move arm back to start position


    // move arm to intermediate position


    // dropping or transporting???


    // finished
    return true;
}

void testTrajectory(ros::NodeHandle node, youbot_proxy::Manipulator& m) {
    // use of a start postion
    // orientation of the gripper relativ to the base coordinate system:
    // x -> front
    // y -> left
    // z -> top
    //                                                                      x,   y,   z
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.2);
    //q.x = 0.5; q.y = 0.5; q.z = 0.5; q.w = 0.5;
    Eigen::Quaterniond grip(0.6851, 0.1749, 0.6851, -0.1749);
    q.x = grip.x();
    q.y = grip.y();
    q.z = grip.z();
    q.w = grip.w();

    tf::Quaternion quat;
    tf::quaternionMsgToTF(q, quat);
    ROS_INFO_STREAM("Axis: " << quat.getAxis().getX() << "/" << quat.getAxis().getY() << "/" << quat.getAxis().getZ() << " Angle: " << quat.getAngle());
    youbot_proxy::Trajectory trajectory = m.newCS2CSTrajectory(0.12, 0.25, 0.05, q);

    // second point
    if(!trajectory.createAndAddPose(-0.12, 0.25, 0.05, q, 0.0)) {
        ROS_ERROR("Unable to create trajectory for 2 poses");
        return;
    }

// ATTENTION: first move arm to start position!!!
    trajectory_msgs::JointTrajectory traj = trajectory.get(true);
    trajectory_msgs::JointTrajectoryPoint point = traj.points.back(); // first point of trajectory
    double firstPt[5];
    int i = 0;
    while (!point.positions.empty()) {
        firstPt[i] = point.positions.back();
        i++;
        point.positions.pop_back();
    }
    brics_actuator::JointPositions msg;
    brics_actuator::JointValue val;
    val.unit = "rad";
    for(int j=0; j<5;j++) {
        std::stringstream ss;
        ss.str("");
        ss << "arm_joint_" << (j+1);
        val.joint_uri = ss.str();
        val.value = firstPt[j];
        msg.positions.push_back(val);
        ROS_ERROR_STREAM("First Point: " << val.joint_uri << " has value " << val.value);
    }
    moveToStartPose(node, m, msg);
    ros::Duration(1).sleep();
    ros::spinOnce();
// ATTENTION end

    // check and correct gripper orientation (yaw)
    double yaw = M_PI_2; // PI/2: gripper is parallel to x
    correctGripperOrientation(yaw, traj);

    if(!m.moveTEST(traj)) {
        ROS_ERROR("CS2CS Trajectory executation failed!");
    }
}

void testTrajectoryWithArmPosition(youbot_proxy::Manipulator& m) {

    // use of the actual arm position:
    youbot_proxy::Trajectory trajectory = m.newSmartTrajectory();

    // add a new key point
    //Eigen::Quaterniond grip(0.6851, 0.1749, 0.6851, -0.1749);
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0, 1.6, 0.0);
    geometry_msgs::Pose kp = youbot_proxy::Trajectory::createPose(0.00, 0.00, 0.80, q);
    if(!trajectory.addPose(kp, youbot_proxy::Trajectory::DEFAULT_MAX_VEL)) {
        ROS_ERROR("Unable to create trajectory for 1. key point");
        return;
    }

/*
    // add another key point
    if(!trajectory.createAndAddPose(0.29, 0.00, -0.1, grip, 0.0)) {
        ROS_ERROR("Unable to create trajectory for 2. key point");
        return;
    }
*/

    // move manipulator
    if(!m.move(trajectory)) {
        ROS_ERROR("Trajectory executation failed!");
    }
}

void testMoveTo(youbot_proxy::Manipulator& m) {
    //Eigen::Quaterniond grip(0.6851, 0.1749, 0.6851, 0.1749);
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0, 1.6, 0.0);

    // alt. 1:
    geometry_msgs::Pose pose = youbot_proxy::Trajectory::createPose(0.27, 0.00, 0.05, q);
    if(!m.moveTo(pose)) {
        ROS_ERROR("Trajectory executation failed!");
    }

    // alt. 2:
    /*
    if(!m.moveTo(0.27, 0.00, 0.05, grip)) {
        ROS_ERROR("Trajectory executation failed!");
    }
    */
}


//----------------------------------- MAIN -----------------------------------------------
int main(int argc, char** argv)
{
    // init node
    ros::init(argc, argv, "cleaner");
    ros::NodeHandle n;
    ros::spinOnce();
    youbot_proxy::Manipulator m(n);
    ros::spinOnce();

    testTrajectory(n, m);
    //testTrajectoryWithArmPosition(m);
    //testMoveTo(m);
}
