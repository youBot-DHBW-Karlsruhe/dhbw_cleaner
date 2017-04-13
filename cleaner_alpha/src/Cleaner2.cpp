#include "cleaner_alpha/CSTrajectoryGenerator.h"
#include "cleaner_alpha/JSTrajectoryGenerator.h"
#include "cleaner_alpha/TrajectoryGeneratorFactory.h"
#include "cleaner_alpha/util/ConstArray.h"
#include "cleaner_alpha/Gripper.h"
#include "cleaner_alpha/Manipulator.h"

#include <math.h>

#include "object_recognition/ObjectPosition.h"

#include "tf/transform_datatypes.h"


namespace youbot_proxy {

#define ARM_POSE_INIT   {0.00004027682889217683, -0.0, -0.00010995574287564276, 0.0008185840012874813, 0.0023451325442290006}
#define ARM_POSE_TOWER  {2.994239875087764, 1.1520884828390492, -2.700355965393107, 2.000221069091924, 2.9442475376037303}
#define ARM_POSE_GRAB   {2.953187717239413, 2.4635926544333344, -1.7269394542799927, 2.8039599388965972, 2.933296211100019}
#define ARM_POSE_DROP   {2.94689446272501, 0.08719933455156284, -2.822768123140233, 0.053185836191759595, 5.855950830183301}


class ObjectDetectionListener {
private:
    ros::Subscriber objectPositionSubscriber;

    geometry_msgs::Pose objectPosition;
    bool available;

public:
    ObjectDetectionListener(ros::NodeHandle node, std::string objectPositionTopic) {
        objectPositionSubscriber = node.subscribe<object_recognition::ObjectPosition>(objectPositionTopic, 5, &ObjectDetectionListener::objectPositionCallback, this);
        available = false;
    }

    void objectPositionCallback(const object_recognition::ObjectPositionConstPtr& msg) {
        //if(msg->object_id != "") {
        //    return;
        //}
        geometry_msgs::Pose pose;
        pose.position = msg->pose.position;
        pose.orientation = msg->pose.orientation;
        objectPosition = pose;
        available = true;
    }

    geometry_msgs::Pose getObjectPosition() {
        available = false;
        return objectPosition;
    }

    bool isAvailable() {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ros::spinOnce();
        return available;
    }
};

}


//----------------------------------- MAIN -----------------------------------------------
int main(int argc, char** argv)
{
    // init node
    ros::init(argc, argv, "cleaner");
    ros::NodeHandle n;

    // create classes
    youbot_proxy::TrajectoryGeneratorFactory tgFactory(n);
    youbot_proxy::Gripper gripper(n, "/arm_1/gripper_controller/position_command");
    youbot_proxy::Manipulator m(n, gripper, tgFactory, "/joint_states", "/torque_control");
    youbot_proxy::ObjectDetectionListener objListener(n, "object_position_single");

    ros::spinOnce();
    ros::spinOnce();

    double observe_pose_far[5] = {3.007873581667766, 1.199917217148509, -2.552009960290597, 1.5187166851994713, 2.933163467748459};
    double observe_pose_near[5] = {2.988721949529536, 1.141102977758708, -1.2992684737451308, 0.33778758193668285, 2.921747539514288};

    // --------------- test move arm to joint pos -----------------------------
    brics_actuator::JointPositions position;
    for(int i=0; i<5; i++) {
        brics_actuator::JointValue val;
        val.joint_uri = m.ARM_JOINT_NAMES[i];
        val.unit = "rad";
        val.value = observe_pose_far[i];
        position.positions.push_back(val);
    }
    m.moveArmToJointPosition(position);
    ros::Duration(2.0).sleep();

    // --------------- test grabbing with object recognition ------------------

    while(ros::ok()) {
        // retrieve object position
        ROS_INFO("Waiting for object detection");
        while(!objListener.isAvailable() && ros::ok()) {}
        geometry_msgs::Pose objectPose = objListener.getObjectPosition();

        // check orientation values
        double roll, pitch, yaw;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(objectPose.orientation, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        ROS_INFO_STREAM("Object Position:\n x=" << objectPose.position.x << ", y=" << objectPose.position.y << ", z=" << objectPose.position.z);
        ROS_INFO_STREAM("Object Orientation:\n Roll=" << roll << ", Pitch=" << pitch << ", Yaw=" << yaw);


        if(pitch != 0) pitch = 0;
        if(yaw != 0) yaw = 0;
        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        objectPose.orientation = q;

        /*
          range in 0.31 > std:sqrt(x² + y²)
         */
        if(m.grabObjectAt(objectPose)) {
            ROS_ERROR("main(): grabbing successful");
            break;
        }
        ROS_ERROR("main(): grabbing failed");
    }
    /*

    // old testcode
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    // examples: (0.12 / 0.25, -0.10), (0.29 / 0.00 / -0.10)
    geometry_msgs::Pose pose = youbot_proxy::TrajectoryGenerator::createPose(0.12, 0.25, -0.10, q);
    if(!m.grabObjectAt(pose)) {
        ROS_ERROR("main(): grabbing failed");
    }
    */
}
