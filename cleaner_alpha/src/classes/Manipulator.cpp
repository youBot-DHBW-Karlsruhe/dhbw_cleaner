#include "ros/ros.h"
#include "torque_control/torque_trajectoryAction.h"
#include "actionlib/client/simple_action_client.h"
#include "trajectory_generator/CStoCS.h"
#include "trajectory_generator/JStoCS.h"
#include "brics_actuator/JointPositions.h"
#include "Eigen/Dense"

namespace youbot_proxy {

class Trajectory {
private:
    ros::ServiceClient cs2csGenerator;

    trajectory_generator::CStoCS cs2cs;
    trajectory_msgs::JointTrajectory t;

    double max_acc;
    double max_vel;

    void resetCS2CS(const geometry_msgs::Pose start, double vel) {
        cs2cs.request.start_pos = start;
        cs2cs.request.start_vel = checkedVel(vel);
        cs2cs.request.max_acc = max_acc;
        cs2cs.request.max_vel = max_vel;
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

    Trajectory(const ros::ServiceClient& generator, const geometry_msgs::Pose start) {
        cs2csGenerator = generator;
        max_acc = DEFAULT_MAX_ACC;
        max_vel = DEFAULT_MAX_ACC;

        resetCS2CS(start, 0.0);
    }

    void setMaxVel(double vel) {
        max_vel = checkedVel(vel);
    }

    void setMaxAcc(double acc) {
        max_acc = checkedAcc(acc);
    }

    bool addPose(const geometry_msgs::Pose p, double vel) {
        cs2cs.request.end_pos = p;
        cs2cs.request.end_vel = checkedVel(vel);

        // generate trajectory points and add to trajectory
        if(cs2csGenerator.call(cs2cs) && cs2cs.response.feasible) {
            trajectory_msgs::JointTrajectory temp = cs2cs.response.trajectory;
            t.joint_names = temp.joint_names;
            trajectory_msgs::JointTrajectoryPoint point;
            while (!temp.points.empty()) {
              point = temp.points.back();
              temp.points.pop_back();
              t.points.insert(t.points.begin(), point);
            }
            // reset cs2cs object to start over at this pose
            resetCS2CS(p, vel);

            return true;
        } else {
            ROS_ERROR("Could not add point to trajectory.");
            return false;
        }
    }

    trajectory_msgs::JointTrajectory get() {
        trajectory_msgs::JointTrajectory temp = t;
        t.joint_names.clear();
        t.points.clear();
        return temp;
    }
};


class Manipulator {

    private:

        ros::Duration timeout;

        ros::ServiceClient cs2csGenerator;
        ros::ServiceClient js2csGenerator;

        ros::Publisher armPublisher;

        actionlib::SimpleActionClient<torque_control::torque_trajectoryAction>* torqueController;

    public:
        // constants
        static const double DEFAULT_TIMEOUT = 20;

        Manipulator(ros::NodeHandle node) {
            timeout = ros::Duration(DEFAULT_TIMEOUT);

            cs2csGenerator = node.serviceClient<trajectory_generator::CStoCS>("CS_to_CS");
            js2csGenerator = node.serviceClient<trajectory_generator::JStoCS>("JS_to_CS");

            armPublisher = node.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command", 1);

            torqueController = new actionlib::SimpleActionClient<torque_control::torque_trajectoryAction>(node, "torque_control", true);
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

        Trajectory newCS2CSTrajectory(geometry_msgs::Pose start) {
            return Trajectory(cs2csGenerator, start);
        }

        void testTrajectory() {
            geometry_msgs::Pose start;
            start.position.x = 0.25;
            start.position.y = 0.00;
            start.position.z = 0.05;
            Eigen::Quaterniond grip(0.6851, 0.1749, 0.6851, -0.1749);
            start.orientation.x = grip.x();
            start.orientation.y = grip.y();
            start.orientation.z = grip.z();
            start.orientation.w = grip.w();

            Trajectory trajectory = newCS2CSTrajectory(start);

            geometry_msgs::Pose kp;
            kp.position.x = 0.25;
            kp.position.y = 0.00;
            kp.position.z = -0.1;
            kp.orientation = start.orientation;
            if(!trajectory.addPose(kp, Trajectory::DEFAULT_MAX_VEL)) {
                ROS_ERROR("Unable to create trajectory for 2 poses");
                return;
            }


            torque_control::torque_trajectoryGoal goal;
            goal.trajectory = trajectory.get();

            // ATTENTION: first move arm to start position!!!
            trajectory_msgs::JointTrajectoryPoint point = goal.trajectory.points.back();
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
                ss << "arm_joint_" << (i+1);
                val.joint_uri = ss.str().c_str();
                val.value = point.positions[i];
                msg.positions.push_back(val);
            }
            armPublisher.publish(msg);

            // execute torque controlled trajectory
            ros::spinOnce();
            ros::Duration(0.5).sleep();

            torqueController->sendGoal(goal);

            // wait for the action to return
            bool finishedBeforeTimeout = torqueController->waitForResult(ros::Duration(20.0));
            if (finishedBeforeTimeout) {
                ROS_INFO("Action finished: %s", torqueController->getState().toString().c_str());
            } else {
                ROS_INFO("Action did not finish before the time out.");
            }
        }

};

}