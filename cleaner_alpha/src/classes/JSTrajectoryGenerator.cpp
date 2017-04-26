#include "cleaner_alpha/JSTrajectoryGenerator.h"

namespace youbot_proxy {

///////////////////////////////////////////////////////////////////////////////
// private methods
///////////////////////////////////////////////////////////////////////////////
brics_actuator::JointVelocities JSTrajectoryGenerator::createJointVelocities(double vel) const {
    brics_actuator::JointVelocities velocityMsg;
    brics_actuator::JointValue v;
    double checkedVelocity = checkedVel(vel);

    v.unit = "s^-1 rad";
    v.value = checkedVelocity;
    for(int j=0; j<5; j++) {
        std::stringstream st;
        st << "arm_joint_" << (j+1);
        v.joint_uri = st.str().c_str();
        velocityMsg.velocities.push_back(v);
    }

    return velocityMsg;
}

void JSTrajectoryGenerator::resetJS2JS(const brics_actuator::JointPositions start, double vel) {
    ++keyPointCounter;

    brics_actuator::JointVelocities velocity = createJointVelocities(vel);

    js2js.request.start_pos = start;
    js2js.request.start_vel = velocity;
    js2js.request.max_acc = max_acc;
    js2js.request.max_vel = max_vel;
}

bool JSTrajectoryGenerator::checkPositions(const brics_actuator::JointPositions& pos) const {
    return pos.positions.size() != 5;
}

///////////////////////////////////////////////////////////////////////////////
// public methods
///////////////////////////////////////////////////////////////////////////////
JSTrajectoryGenerator::JSTrajectoryGenerator(const ros::ServiceClient& js2jsGen, const brics_actuator::JointPositions startPosition) {
    js2jsGenerator = js2jsGen;
    max_acc = TrajectoryGenerator::DEFAULT_MAX_ACC;
    max_vel = TrajectoryGenerator::DEFAULT_MAX_VEL;

    if(!js2jsGenerator.isValid()) {
        ROS_ERROR("JS to JS TrajectoryGenerator not valid!");
    }

    if(checkPositions(startPosition)) {
        ROS_WARN("Empty start position");
    }
    resetJS2JS(startPosition, 0.0);

    keyPointCounter = 1;
    valid = false;
}

bool JSTrajectoryGenerator::addPosition(const brics_actuator::JointPositions p, double vel) {
    if(checkPositions(p)) {
        ROS_WARN("Empty position");
        return false;
    }
    // use cs2cs
    js2js.request.end_pos = p;
    js2js.request.end_vel = createJointVelocities(vel);

    if(this->callGenerator<trajectory_generator::JStoJS>(js2jsGenerator, js2js)) {
        // reset cs2cs object to start over at this pose
        resetJS2JS(p, vel);
        valid = true;
        return true;
    }
    return false;
}

}

