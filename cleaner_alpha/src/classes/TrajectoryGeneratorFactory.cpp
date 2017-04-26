#include "cleaner_alpha/TrajectoryGeneratorFactory.h"

namespace youbot_proxy {

///////////////////////////////////////////////////////////////////////////////
// public methods
///////////////////////////////////////////////////////////////////////////////
TrajectoryGeneratorFactory::TrajectoryGeneratorFactory(ros::NodeHandle node) {
    cs2csGenerator = node.serviceClient<trajectory_generator::CStoCS>("/From_CS_to_CS");
    js2jsGenerator = node.serviceClient<trajectory_generator::JStoJS>("/From_JS_to_JS");
}

CSTrajectoryGenerator TrajectoryGeneratorFactory::getCSTrajectoryGenerator(double x, double y, double z, geometry_msgs::Quaternion q) const {
    return getCSTrajectoryGenerator(TrajectoryGenerator::createPose(x, y, z, q));
}

CSTrajectoryGenerator TrajectoryGeneratorFactory::getCSTrajectoryGenerator(const geometry_msgs::Pose start) const {
    return CSTrajectoryGenerator(cs2csGenerator, start);
}

JSTrajectoryGenerator TrajectoryGeneratorFactory::getJSTrajectoryGenerator(const brics_actuator::JointPositions start) const {
    return JSTrajectoryGenerator(js2jsGenerator, start);
}

}

