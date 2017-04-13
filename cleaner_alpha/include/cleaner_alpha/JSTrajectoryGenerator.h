#ifndef JSTRAJECTORY_GENERATOR_H
#define JSTRAJECTORY_GENERATOR_H

#include "cleaner_alpha/TrajectoryGenerator.h"

namespace youbot_proxy {

class JSTrajectoryGenerator: public TrajectoryGenerator {
private:
    ros::ServiceClient js2jsGenerator;

    trajectory_generator::JStoJS js2js;

    brics_actuator::JointVelocities createJointVelocities(double vel) const;

    bool checkPositions(const brics_actuator::JointPositions& pos) const;

    void resetJS2JS(const brics_actuator::JointPositions start, double vel);

public:

    JSTrajectoryGenerator(const ros::ServiceClient& js2jsGen, const brics_actuator::JointPositions startPosition);

    bool addPosition(const brics_actuator::JointPositions p, double vel = 0.0);
};

}

#endif //JSTRAJECTORY_GENERATOR_H
