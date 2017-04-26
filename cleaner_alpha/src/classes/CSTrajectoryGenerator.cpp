#include "cleaner_alpha/CSTrajectoryGenerator.h"

namespace youbot_proxy {

///////////////////////////////////////////////////////////////////////////////
// private methods
///////////////////////////////////////////////////////////////////////////////
void CSTrajectoryGenerator::resetCS2CS(const geometry_msgs::Pose start, double vel) {
    ++keyPointCounter;

    cs2cs.request.start_pos = start;
    cs2cs.request.start_vel = checkedVel(vel);
    cs2cs.request.max_acc = max_acc;
    cs2cs.request.max_vel = max_vel;
}

///////////////////////////////////////////////////////////////////////////////
// public methods
///////////////////////////////////////////////////////////////////////////////
CSTrajectoryGenerator::CSTrajectoryGenerator(const ros::ServiceClient& cs2csGen, const geometry_msgs::Pose startPose) {
    cs2csGenerator = cs2csGen;
    max_acc = TrajectoryGenerator::DEFAULT_MAX_ACC;
    max_vel = TrajectoryGenerator::DEFAULT_MAX_VEL;

    if(!cs2csGenerator.isValid()) {
        ROS_ERROR("JS to JS TrajectoryGenerator not valid!");
    }

    resetCS2CS(startPose, 0.0);

    keyPointCounter = 1;
    valid = false;
}

bool CSTrajectoryGenerator::createAndAddPose(double x, double y, double z, const geometry_msgs::Quaternion gripperOrientation, double vel) {
    geometry_msgs::Pose p = TrajectoryGenerator::createPose(x, y, z, gripperOrientation);
    return addPose(p, vel);
}

bool CSTrajectoryGenerator::addPose(const geometry_msgs::Pose p, double vel) {
    cs2cs.request.end_pos = p;
    cs2cs.request.end_vel = checkedVel(vel);

    if(this->callGenerator<trajectory_generator::CStoCS>(cs2csGenerator, cs2cs)) {
        // reset cs2cs object to start over at this pose
        resetCS2CS(p, vel);
        valid = true;
        return true;
    }
    return false;
}

}

