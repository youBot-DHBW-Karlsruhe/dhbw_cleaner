#ifndef POSE_H
#define POSE_H

#include "ros/ros.h"
#include "brics_actuator/JointPositions.h"

namespace youbot_proxy {

namespace util {

#define ARM_POSE_INIT    {1.0069207223044208e-05, 1.0069207223044208e-05, -0.0009896016858807848, 0.0012389379478945663, 0.0028318581666161515}
#define ARM_POSE_TOWER   {2.952, 1.1520884828390492, -2.700355965393107, 2.000221069091924, 2.94}
//#define ARM_POSE_GRAB    {2.952, 2.356879196283512, -1.72240958825714, 2.704889150848885, 2.94}
#define ARM_POSE_DROP    {2.952, 0.48282855555219284, -3.2523180866655657, 0.6907521537350741, 2.94}
//old: #define ARM_POSE_OBSERVE_FAR {3.007873581667766, 1.199917217148509, -2.552009960290597, 1.5187166851994713, 2.933163467748459}
//#define ARM_POSE_OBSERVE_FAR {0.123967267645879418, 0.9867017542005481, -2.4045593090943598, 2.039004251639409, 2.6552873749596088}
#define ARM_POSE_OBSERVE_FAR {0.003322838383604589, 1.1402773027664184, -2.498335849804015, 2.212013086506115, 2.7569024207685304}
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

    Pose(const std::string* joint_names, size_t n);

public:
    static const Pose createPose(const std::string* jointNameArray, int size);

    void addPose(POSE_ID poseId, double j1, double j2, double j3, double j4, double j5);

    void addPose(POSE_ID poseId, double* pose);

    const std::vector<double> pose(POSE_ID poseId) const;

    const brics_actuator::JointPositions jointPositions(POSE_ID poseId) const;
};

}
}

#endif // POSE_H
