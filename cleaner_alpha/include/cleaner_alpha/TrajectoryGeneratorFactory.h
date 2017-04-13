#ifndef TRAJECTORY_GENERATOR_FACTORY_H
#define TRAJECTORY_GENERATOR_FACTORY_H

#include "cleaner_alpha/CSTrajectoryGenerator.h"
#include "cleaner_alpha/JSTrajectoryGenerator.h"

namespace youbot_proxy {

class TrajectoryGeneratorFactory {
private:

    ros::ServiceClient cs2csGenerator;
    ros::ServiceClient js2jsGenerator;

public:
    TrajectoryGeneratorFactory(ros::NodeHandle node);

    CSTrajectoryGenerator getCSTrajectoryGenerator(double x, double y, double z, geometry_msgs::Quaternion q) const;

    CSTrajectoryGenerator getCSTrajectoryGenerator(const geometry_msgs::Pose start) const;

    JSTrajectoryGenerator getJSTrajectoryGenerator(const brics_actuator::JointPositions start) const;

};

}

#endif // TRAJECTORY_GENERATOR_FACTORY_H
