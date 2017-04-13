#ifndef CSTRAJECTORY_GENERATOR_H
#define CSTRAJECTORY_GENERATOR_H

#include "cleaner_alpha/TrajectoryGenerator.h"

namespace youbot_proxy {

class CSTrajectoryGenerator: public TrajectoryGenerator {
private:
    ros::ServiceClient cs2csGenerator;
    trajectory_generator::CStoCS cs2cs;

    void resetCS2CS(const geometry_msgs::Pose start, double vel);

public:

    CSTrajectoryGenerator(const ros::ServiceClient& cs2csGen, const geometry_msgs::Pose startPose);

    bool createAndAddPose(double x, double y, double z, const geometry_msgs::Quaternion gripperOrientation, double vel = 0.0);

    bool addPose(const geometry_msgs::Pose p, double vel = 0.0);
};

}

#endif //CSTRAJECTORY_GENERATOR_H
