#ifndef POSE_H
#define POSE_H

#include "ros/ros.h"
#include "brics_actuator/JointPositions.h"

namespace youbot_proxy {

namespace util {

#define ARM_POSE_INIT    {0.00004027682889217683, -0.0, -0.00010995574287564276, 0.0008185840012874813, 0.0023451325442290006}
#define ARM_POSE_TOWER   {2.952, 1.1520884828390492, -2.700355965393107, 2.000221069091924, 2.94}
//#define ARM_POSE_GRAB    {2.952, 2.356879196283512, -1.72240958825714, 2.704889150848885, 2.94}
#define ARM_POSE_DROP    {2.952, 0.48282855555219284, -3.2523180866655657, 0.6907521537350741, 2.94}
#define ARM_POSE_OBSERVE_FAR {3.007873581667766, 1.199917217148509, -2.552009960290597, 1.5187166851994713, 2.933163467748459}
#define ARM_POSE_OBSERVE_NEAR {2.988721949529536, 1.141102977758708, -1.2992684737451308, 0.33778758193668285, 2.921747539514288}

class Pose {
public:
    enum POSE_ID {
        INIT = 0, OBSERVE_NEAR = 1, OBSERVE_FAR = 2, DROP_AT_PLATE = 3, TOWER = 4
    };

private:
    const std::string* ARM_JOINT_NAMES;
    const size_t N;
    std::map<POSE_ID, double*> poses;
    std::map<POSE_ID, brics_actuator::JointPositions> joint_positions;

public:
    Pose(const std::string* joint_names, size_t n, double* defaultPose);

    void addPose(POSE_ID poseId, double j1, double j2, double j3, double j4, double j5);

    void addPose(POSE_ID poseId, double* pose);

    const std::vector<double> pose(POSE_ID poseId) const;

    const brics_actuator::JointPositions jointPositions(POSE_ID poseId) const;
};

}
}

#endif // POSE_H
