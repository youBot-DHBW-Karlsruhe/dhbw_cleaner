#include "ros/ros.h"
#include "torque_control/torque_trajectoryAction.h"
#include "actionlib/client/simple_action_client.h"
#include "trajectory_generator/CStoCS.h"
#include "trajectory_generator/JStoCS.h"
#include "brics_actuator/JointPositions.h"
#include "sensor_msgs/JointState.h"
#include "Eigen/Dense"

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

    static const geometry_msgs::Pose createPose(double x, double y, double z, const Eigen::Quaterniond gripperOrientation) {
        geometry_msgs::Pose kp;
        kp.position.x = x;
        kp.position.y = y;
        kp.position.z = z;
        kp.orientation.x = gripperOrientation.x();
        kp.orientation.y = gripperOrientation.y();
        kp.orientation.z = gripperOrientation.z();
        kp.orientation.w = gripperOrientation.w();
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

    bool createAndAddPose(double x, double y, double z, const Eigen::Quaterniond gripperOrientation, double vel) {
        geometry_msgs::Pose p = Trajectory::createPose(x, y, z, gripperOrientation);
        return addPose(p, vel);
    }

    bool addPose(const geometry_msgs::Pose p, double vel) {
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

        /*
         old implementation:
            // use cs2cs
            cs2cs.request.end_pos = p;
            cs2cs.request.end_vel = checkedVel(vel);

            // generate trajectory points and add to trajectory
            if(cs2csGenerator.call(cs2cs)) {
                feasible = cs2cs.response.feasible;
                if(feasible) {
                    trajectory_msgs::JointTrajectory temp = cs2cs.response.trajectory;
                    trajectory_msgs::JointTrajectoryPoint point;
                    while (!temp.points.empty()) {
                        point = temp.points.back();
                        temp.points.pop_back();
                        t.points.insert(t.points.begin(), point);
                    }
                    t.joint_names = temp.joint_names;
                    // reset cs2cs object to start over at this pose
                    resetCS2CS(p, vel);

                    return true;
                } else {
                    ROS_ERROR("Could not add point to trajectory. Trajectory not feasible!");
                    return false;
                }
            } else {
                ROS_ERROR("Could not add point to trajectory. Call to trajectory generator failed!");
                return false;
            }
        */
    }

    trajectory_msgs::JointTrajectory get() {
        ROS_INFO_STREAM("Returning trajectory from " << (keyPointCounter+1) << " key points");
        trajectory_msgs::JointTrajectory temp = t;
        t.joint_names.clear();
        t.points.clear();
        keyPointCounter = 1;

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

        ros::Publisher armPublisher;
        ros::Subscriber armPositionSubscriber;

        actionlib::SimpleActionClient<torque_control::torque_trajectoryAction>* torqueController;

    public:
        // constants
        static const double DEFAULT_TIMEOUT = 20;

        Manipulator(ros::NodeHandle node) {
            timeout = ros::Duration(DEFAULT_TIMEOUT);

            cs2csGenerator = node.serviceClient<trajectory_generator::CStoCS>("From_CS_to_CS");
            js2csGenerator = node.serviceClient<trajectory_generator::JStoCS>("From_JS_to_CS");

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

        Trajectory newCS2CSTrajectory(double x, double y, double z, const Eigen::Quaterniond gripperOrientation) const {
            return Trajectory(cs2csGenerator, Trajectory::createPose(x, y, z, gripperOrientation));
        }

        Trajectory newCS2CSTrajectory(const geometry_msgs::Pose start) const {
            return Trajectory(cs2csGenerator, start);
        }

        Trajectory newSmartTrajectory() const {
            sensor_msgs::JointState jointState = getJointStates();
            brics_actuator::JointPositions jointPositions = Trajectory::jointStateToJointPositions(jointState);
            return Trajectory(cs2csGenerator, js2csGenerator, jointPositions);
        }

        void armPositionHandler(const sensor_msgs::JointStateConstPtr& msg) {
            jointState = *msg;
        }

        sensor_msgs::JointState getJointStates() const {
            ros::spinOnce();
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

        bool moveTo(double x, double y, double z, const Eigen::Quaterniond gripperOrientation) {
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

}

//-------------------------------- FUNCTIONS ---------------------------------------------
void testTrajectory(youbot_proxy::Manipulator& m) {
    // TODO: use actual arm position as start position
    // --> needs JS2CS --> see testTrajectoryWithArmPosition()

    // use of a start postion
    Eigen::Quaterniond grip(0.6851, 0.1749, 0.6851, -0.1749);
    youbot_proxy::Trajectory trajectory = m.newCS2CSTrajectory(0.27, 0.00, 0.05, grip);

    // second point
    if(!trajectory.createAndAddPose(0.27, 0.00, -0.10, grip, 0.0)) {
        ROS_ERROR("Unable to create trajectory for 2 poses");
        return;
    }

// ATTENTION: first move arm to start position!!!
    trajectory_msgs::JointTrajectory traj = trajectory.get();
    trajectory_msgs::JointTrajectoryPoint point = traj.points.back();
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
        ss << "arm_joint_" << (j+1);
        val.joint_uri = ss.str().c_str();
        val.value = point.positions[j];
        msg.positions.push_back(val);
        ROS_ERROR_STREAM("First Point: " << val.joint_uri << " has value " << val.value);
    }
    m.publish(msg);
// ATTENTION end


    if(!m.move(trajectory)) {
        ROS_ERROR("Trajectory executation failed!");
    }
}

void testTrajectoryWithArmPosition(youbot_proxy::Manipulator& m) {

    // use of the actual arm position:
    youbot_proxy::Trajectory trajectory = m.newSmartTrajectory();

    // add a new key point
    Eigen::Quaterniond grip(0.6851, 0.1749, 0.6851, -0.1749);
    geometry_msgs::Pose kp = youbot_proxy::Trajectory::createPose(0.27, 0.00, 0.05, grip);
    if(!trajectory.addPose(kp, youbot_proxy::Trajectory::DEFAULT_MAX_VEL)) {
        ROS_ERROR("Unable to create trajectory for 1. key point");
        return;
    }

    // add another key point
    if(!trajectory.createAndAddPose(0.27, 0.00, -0.1, grip, 0.0)) {
        ROS_ERROR("Unable to create trajectory for 2. key point");
        return;
    }

    // move manipulator
    if(!m.move(trajectory)) {
        ROS_ERROR("Trajectory executation failed!");
    }
}

void testMoveTo(youbot_proxy::Manipulator& m) {
    Eigen::Quaterniond grip(0.6851, 0.1749, 0.6851, -0.1749);

    // alt. 1:
    geometry_msgs::Pose pose = youbot_proxy::Trajectory::createPose(0.27, 0.00, 0.05, grip);
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

    //testTrajectory(m);
    testTrajectoryWithArmPosition(m);
    //testMoveTo(m);
}
