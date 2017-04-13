#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include "ros/ros.h"

#include "brics_actuator/JointPositions.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"

#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_generator/CStoCS.h"
#include "trajectory_generator/JStoJS.h"

namespace youbot_proxy {

class TrajectoryGenerator {
///////////////////////////////////////////////////////////////////////////////
// protected methods
///////////////////////////////////////////////////////////////////////////////
protected:
    trajectory_msgs::JointTrajectory t;
    double max_acc;
    double max_vel;

    bool valid;
    int keyPointCounter; // beginning at 1 = start position

    double checkedVel(double vel) const {
        if(vel > 0.199999) {
            return 0.199999;
        } else if(vel <= .0) {
            return 0.000001;
        } else {
            return vel;
        }
    }


    double checkedAcc(double acc) const {
        if(acc > 0.999999) {
            return 0.999999;
        } else if (acc <= .0) {
            return 0.000001;
        } else {
            return acc;
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

    template <typename T> // T is either   JStoJS or CStoCS   of namespace trajectory_generator
    bool callGenerator(ros::ServiceClient& generator, T& msg) {     // !! no const
        BOOST_STATIC_ASSERT(((boost::is_base_of<trajectory_generator::JStoJS, T>::value) || (boost::is_base_of<trajectory_generator::CStoCS, T>::value))); //Yes, the double parentheses are needed, otherwise the comma will be seen as macro argument separator

        bool feasible = false;
        if(generator.call(msg)) {
            feasible = msg.response.feasible;
            if(feasible) {
                // extract trajectory points and append them to member 't'
                extractAndStoreTrajectory(msg.response.trajectory);
                return true;
            } else {
                ROS_ERROR("TG: Could not add point to trajectory. Trajectory not feasible!");
                return false;
            }
        } else {
            ROS_ERROR("TG: Could not add point to trajectory. Call to trajectory generator failed!");
            return false;
        }
    }

///////////////////////////////////////////////////////////////////////////////
// public methods
///////////////////////////////////////////////////////////////////////////////
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

    static void reverseTrajectory(trajectory_msgs::JointTrajectory& trajectory) {
        std::reverse(trajectory.points.begin(), trajectory.points.end());
    }

    void setMaxVel(double vel) {
        max_vel = checkedVel(vel);
    }

    void setMaxAcc(double acc) {
        max_acc = checkedAcc(acc);
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

}

#endif //TRAJECTORY_GENERATOR_H
