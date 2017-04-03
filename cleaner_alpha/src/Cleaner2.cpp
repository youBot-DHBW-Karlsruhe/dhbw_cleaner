#include "ros/ros.h"

#include <math.h>
#include <boost/assign/list_of.hpp>
#include "Eigen/Dense"

#include "actionlib/client/simple_action_client.h"
#include "torque_control/torque_trajectoryAction.h"
#include "trajectory_generator/CStoCS.h"
#include "trajectory_generator/JStoCS.h"
#include "trajectory_generator/JStoJS.h"

#include "control_msgs/FollowJointTrajectoryAction.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "brics_actuator/JointPositions.h"
#include "sensor_msgs/JointState.h"

#include "object_recognition/ObjectPosition.h"

#include "tf/transform_datatypes.h"


namespace youbot_proxy {

#define ARM_POSE_INIT   {0.00004027682889217683, -0.0, -0.00010995574287564276, 0.0008185840012874813, 0.0023451325442290006}
#define ARM_POSE_TOWER  {2.994239875087764, 1.1520884828390492, -2.700355965393107, 2.000221069091924, 2.9442475376037303}
#define ARM_POSE_GRAB   {2.953187717239413, 2.4635926544333344, -1.7269394542799927, 2.8039599388965972, 2.933296211100019}
#define ARM_POSE_DROP   {2.94689446272501, 0.08719933455156284, -2.822768123140233, 0.053185836191759595, 5.855950830183301}

class TrajectoryGenerator {
protected:
    trajectory_msgs::JointTrajectory t;
    double max_acc;
    double max_vel;

    bool valid;
    int keyPointCounter; // beginning at 1 = start position

    double checkedVel(double vel) {
        if(vel > 0.199999) {
            return 0.199999;
        } else if(vel <= .0) {
            return 0.000001;
        } else {
            return vel;
        }
    }

    double checkedAcc(double acc) {
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

class JSTrajectoryGenerator: public TrajectoryGenerator {
private:
    ros::ServiceClient js2jsGenerator;

    trajectory_generator::JStoJS js2js;

    brics_actuator::JointVelocities createJointVelocities(double vel) {
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

    void resetJS2JS(const brics_actuator::JointPositions start, double vel) {
        ++keyPointCounter;

        brics_actuator::JointVelocities velocity = createJointVelocities(vel);

        js2js.request.start_pos = start;
        js2js.request.start_vel = velocity;
        js2js.request.max_acc = max_acc;
        js2js.request.max_vel = max_vel;
    }

    bool checkPositions(const brics_actuator::JointPositions& pos) {
        return pos.positions.size() != 5;
    }

public:

    JSTrajectoryGenerator(const ros::ServiceClient& js2jsGen, const brics_actuator::JointPositions startPosition) {
        js2jsGenerator = js2jsGen;
        max_acc = TrajectoryGenerator::DEFAULT_MAX_ACC;
        max_vel = TrajectoryGenerator::DEFAULT_MAX_VEL;

        if(checkPositions(startPosition)) {
            ROS_WARN("Empty start position");
        }
        resetJS2JS(startPosition, 0.0);

        keyPointCounter = 1;
        valid = false;
    }

    bool addPosition(const brics_actuator::JointPositions p, double vel = 0.) {
        if(checkPositions(p)) {
            ROS_WARN("Empty position");
            return false;
        }
        // use cs2cs
        js2js.request.end_pos = p;
        js2js.request.end_vel = createJointVelocities(vel);

        if(callGenerator(js2jsGenerator, js2js)) {
            // reset cs2cs object to start over at this pose
            resetJS2JS(p, vel);
            valid = true;
            return true;
        }
        return false;
    }
};


class CSTrajectoryGenerator: public TrajectoryGenerator {
private:
    ros::ServiceClient cs2csGenerator;
    trajectory_generator::CStoCS cs2cs;

    void resetCS2CS(const geometry_msgs::Pose start, double vel) {
        ++keyPointCounter;

        cs2cs.request.start_pos = start;
        cs2cs.request.start_vel = checkedVel(vel);
        cs2cs.request.max_acc = max_acc;
        cs2cs.request.max_vel = max_vel;
    }

public:

    CSTrajectoryGenerator(const ros::ServiceClient& cs2csGen, const geometry_msgs::Pose startPose) {
        cs2csGenerator = cs2csGen;
        max_acc = TrajectoryGenerator::DEFAULT_MAX_ACC;
        max_vel = TrajectoryGenerator::DEFAULT_MAX_VEL;

        resetCS2CS(startPose, 0.0);

        keyPointCounter = 1;
        valid = false;
    }

    bool createAndAddPose(double x, double y, double z, const geometry_msgs::Quaternion gripperOrientation, double vel) {
        geometry_msgs::Pose p = TrajectoryGenerator::createPose(x, y, z, gripperOrientation);
        return addPose(p, vel);
    }

    bool addPose(const geometry_msgs::Pose p, double vel = 0.) {
        cs2cs.request.end_pos = p;
        cs2cs.request.end_vel = checkedVel(vel);

        if(callGenerator(cs2csGenerator, cs2cs)) {
            // reset cs2cs object to start over at this pose
            resetCS2CS(p, vel);
            valid = true;
            return true;
        }
        return false;
    }
};

template <class T, size_t MAX_LENGTH>
class ConstArrayFiller {
private:
    size_t length;

protected:
    void addElement(T* array, const T& element) {
        if(length >= MAX_LENGTH) {
            throw 0;
        }
        array[length] = element;
        length++;
    }

public:
    ConstArrayFiller(): length(0) {}
    ~ConstArrayFiller(){}

    virtual void fill(T* array) = 0;

};

template <class T, size_t MAX_LENGTH>
class ConstArray {
private:
    T array[MAX_LENGTH];

public:
    explicit ConstArray(ConstArrayFiller<T, MAX_LENGTH>& filler) {
        filler.fill(array);
    }

    T const& operator[](size_t i) const {
        assert(i >= 0 && i < MAX_LENGTH);
        return array[i];
    }

    size_t size() const {
        return MAX_LENGTH;
    }

    const T* getArray() const {
        return array;
    }
};


class Manipulator {

    private:
        class ConstJointNameArrayFiller: public ConstArrayFiller<std::string, 5> {
        public:
            virtual void fill(std::string* array) {
                addElement(array, "arm_joint_1");
                addElement(array, "arm_joint_2");
                addElement(array, "arm_joint_3");
                addElement(array, "arm_joint_4");
                addElement(array, "arm_joint_5");
            }
        };
        static ConstJointNameArrayFiller armJointArrayFiller;

        class ConstGripperNameArrayFiller: public ConstArrayFiller<std::string, 2> {
        public:
            virtual void fill(std::string* array) {
                addElement(array, "gripper_finger_joint_l");
                addElement(array, "gripper_finger_joint_r");
            }
        };
        static ConstGripperNameArrayFiller gripperJointArrayFiller;

        typedef ConstArray<std::string, 5> ConstArmJointNameArray;
        typedef ConstArray<std::string, 2> ConstGripperJointNameArray;

        ros::Duration timeout;
        sensor_msgs::JointState jointState;

        ros::ServiceClient cs2csGenerator;
        ros::ServiceClient js2jsGenerator;

        ros::Publisher gripperPublisher;

        ros::Subscriber armPositionSubscriber;

        actionlib::SimpleActionClient<torque_control::torque_trajectoryAction>* torqueController;

        // public for testing only!!!!!! FIXME: remove
    public:
        brics_actuator::JointPositions extractFirstPointPositions(const trajectory_msgs::JointTrajectory& traj) const {
            // extract joint positions and names
            const trajectory_msgs::JointTrajectoryPoint point = traj.points.back(); // first point of trajectory
            const std::vector<std::string> joint_names = traj.joint_names;

            // create joint position message
            brics_actuator::JointPositions jointPositions;
            brics_actuator::JointValue val;

            // TODO: optimize with JOINT_NAMES-array
            for(int i=0; i<ARM_JOINT_NAMES.size();i++) {
                val.joint_uri = joint_names[i];
                val.unit = "rad";
                val.value = point.positions[i];
                jointPositions.positions.push_back(val);
            }

            return jointPositions;
        }

        void correctGripperOrientationConst(double angle, trajectory_msgs::JointTrajectory& traj) const {
            ROS_INFO("Correcting gripper orientation ...");

            // find joint_5 index
            double nPoints = traj.points.size();
            double nJoints = ARM_JOINT_NAMES.size();
            int joint5 = 0;

            for(int i=0; i<nJoints; i++) {
                if(traj.joint_names[i] == ARM_JOINT_NAMES[I_JOINT_5]) {
                    joint5 = i;
                    break;
                }
            }

            // set angle for joint 5 to _angle_ for whole trajectory
            for(int i=0; i<nPoints; i++) {
                trajectory_msgs::JointTrajectoryPoint* point = &traj.points[i];
                if(point->positions.size() == nJoints)     point->positions[joint5] = angle;
                if(point->velocities.size() == nJoints)    point->velocities[joint5] = 0;
                if(point->accelerations.size() == nJoints) point->accelerations[joint5] = 0;
                if(point->effort.size() == nJoints)        point->effort[joint5] = 0;
            }

            ROS_INFO("... finished.");
        }

        bool move(const trajectory_msgs::JointTrajectory& traj ) {
            torque_control::torque_trajectoryGoal goal;

            // delay execution a bit to allow callbacks
            ros::spinOnce();
            ros::Duration(0.5).sleep();
            ros::spinOnce();

            goal.trajectory = traj;
            torqueController->sendGoal(goal);

            // wait for the action to return
            bool finishedBeforeTimeout = torqueController->waitForResult(ros::Duration(20.0));
            if (finishedBeforeTimeout) {
                ROS_INFO("Action finished: %s", torqueController->getState().toString().c_str());
                return true;
            } else {
                ROS_INFO("Action did not finish before the time out.");
                return false;
            }
        }

        brics_actuator::JointPositions createGripperPositionMsg(double left, double right) const {
            brics_actuator::JointPositions msg;
            brics_actuator::JointValue leftGripperValue;
            brics_actuator::JointValue rightGripperValue;

            leftGripperValue.joint_uri = GRIPPER_JOINT_NAMES[I_FINGER_LEFT];
            leftGripperValue.unit = "m";
            leftGripperValue.value = left;

            rightGripperValue.joint_uri = GRIPPER_JOINT_NAMES[I_FINGER_RIGHT];
            rightGripperValue.unit = "m";
            rightGripperValue.value = right;

            msg.positions.push_back(leftGripperValue);
            msg.positions.push_back(rightGripperValue);
            return msg;
        }

        void sendGripperPositionMsg(const brics_actuator::JointPositions msg) {
            this->gripperPublisher.publish(msg);
        }

    public:
        // constants
        const double DEFAULT_TIMEOUT;
        const int I_JOINT_1;
        const int I_JOINT_2;
        const int I_JOINT_3;
        const int I_JOINT_4;
        const int I_JOINT_5;
        const int I_FINGER_LEFT;
        const int I_FINGER_RIGHT;
        const ConstArmJointNameArray ARM_JOINT_NAMES;
        const ConstGripperJointNameArray GRIPPER_JOINT_NAMES;

        Manipulator(ros::NodeHandle node):
            DEFAULT_TIMEOUT(20),
            I_JOINT_1(0),
            I_JOINT_2(1),
            I_JOINT_3(2),
            I_JOINT_4(3),
            I_JOINT_5(4),
            I_FINGER_LEFT(0),
            I_FINGER_RIGHT(1),
            ARM_JOINT_NAMES(armJointArrayFiller),
            GRIPPER_JOINT_NAMES(gripperJointArrayFiller)
        {
            timeout = ros::Duration(DEFAULT_TIMEOUT);

            cs2csGenerator = node.serviceClient<trajectory_generator::CStoCS>("From_CS_to_CS");
            js2jsGenerator = node.serviceClient<trajectory_generator::JStoJS>("From_JS_to_JS");

            gripperPublisher = node.advertise<brics_actuator::JointPositions>("/arm_1/gripper_controller/position_command", 5);

            armPositionSubscriber = node.subscribe<sensor_msgs::JointState>("/joint_states", 1, &Manipulator::armPositionHandler, this);

            torqueController = new actionlib::SimpleActionClient<torque_control::torque_trajectoryAction>("torque_control", true);
            ROS_INFO_STREAM("Waiting " << timeout << " seconds for action server to start");
            torqueController->waitForServer(timeout);
            if(!torqueController->isServerConnected()) {
                ROS_ERROR("Initialization failed: torque controller is not available");
                EXIT_FAILURE;
            }
        }

        ~Manipulator() {
            delete torqueController;
        }

        CSTrajectoryGenerator getCSTrajectoryGenerator(double x, double y, double z, geometry_msgs::Quaternion q) const {
            return getCSTrajectoryGenerator(TrajectoryGenerator::createPose(x, y, z, q));
        }

        CSTrajectoryGenerator getCSTrajectoryGenerator(const geometry_msgs::Pose start) const {
            return CSTrajectoryGenerator(cs2csGenerator, start);
        }

        JSTrajectoryGenerator getJSTrajectoryGenerator(const brics_actuator::JointPositions start) const {
            return JSTrajectoryGenerator(js2jsGenerator, start);
        }

        void armPositionHandler(const sensor_msgs::JointStateConstPtr& msg) {
            sensor_msgs::JointState state;

            // only save arm joint values
            for(int i=0; i<ARM_JOINT_NAMES.size(); i++) {
                if(ARM_JOINT_NAMES[i] != msg->name[i]) {
                    return;
                }
                state.name.push_back(ARM_JOINT_NAMES[i]);
                state.position.push_back(msg->position[i]);
                state.velocity.push_back(msg->velocity[i]);
                state.effort.push_back(msg->effort[i]);
            }

            state.header = msg->header;
            jointState = state;
        }

        sensor_msgs::JointState getJointStates() const {
            ros::spinOnce();
            ros::Duration(0.5).sleep();
            ros::spinOnce();
            return jointState;
        }

        void openGripper() {
            brics_actuator::JointPositions gripperPositionMsg = this->createGripperPositionMsg(0.0115, 0.0115);
            this->sendGripperPositionMsg(gripperPositionMsg);
            ROS_INFO("Gripper opened");
        }

        void closeGripper() {
            brics_actuator::JointPositions gripperPositionMsg = this->createGripperPositionMsg(0.0, 0.0);
            this->sendGripperPositionMsg(gripperPositionMsg);
            ROS_INFO("Gripper closed");
        }


        bool moveArmToJointPosition(const brics_actuator::JointPositions& targetPosition) {
            ROS_INFO("Generating trajectory to joint position (js2js)");
            // extract current joint positions
            sensor_msgs::JointState jointState = this->getJointStates();
            brics_actuator::JointPositions currentJointPositions = youbot_proxy::CSTrajectoryGenerator::jointStateToJointPositions(jointState);

            // create trajectory from current joint position to target position
            // js2js
            trajectory_msgs::JointTrajectory js2jsTraj;
            youbot_proxy::JSTrajectoryGenerator trajectoryGenJs = this->getJSTrajectoryGenerator(currentJointPositions);
            if(!trajectoryGenJs.addPosition(targetPosition)) {
                ROS_ERROR("Could not create trajectory to target position");
                return false;
            }
            js2jsTraj = trajectoryGenJs.get();

            // move arm
            ROS_INFO("Moving arm to target position (js2js)");
            if(!this->move(js2jsTraj)) {
                ROS_ERROR("Could not moarm_joint_2ve arm to target position. Execution of js2js trajectory failed.");
                return false;
            }
            return true;
        }

        bool grabObjectAt(const geometry_msgs::Pose& pose) {
            // configuration
            double dGrabLine = 0.1;
            double gripperExt = 0; // TODO: test
            double joint_5_min = 0.1221730476;
            double joint_1_Aty0 = 2.9499152248919236;

            // extract rpy
            double roll, pitch, yaw;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(pose.orientation, quat);
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            ROS_INFO_STREAM("Roll=" << roll << ", Pitch=" << pitch << ", Yaw=" << yaw);

            // check roll, pitch and yaw
            if(roll < -M_PI || roll > M_PI) {
                ROS_ERROR("Roll must not be within ]-PI, PI[");
                return false;
            }
            if(pitch != 0) {
                ROS_ERROR("Pitch must be 0");
                return false;
            }
            if(yaw != 0) {
                ROS_ERROR("Yaw must be 0");
                return false;
            }

            ///////////////////////////////////////////////////////////////////////////
            // setup phase
            ///////////////////////////////////////////////////////////////////////////
            ROS_INFO("Generating linear trajectory (cs2cs)");
            geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0, M_PI_2, 0.0);

            // create IK start position
            geometry_msgs::Pose armStartPosition;
            armStartPosition.orientation = q;
            armStartPosition.position.x = pose.position.x;
            armStartPosition.position.y = pose.position.y;
            armStartPosition.position.z = pose.position.z + gripperExt + dGrabLine;


            // create IK goal position
            geometry_msgs::Pose armGoalPosition;
            armGoalPosition.orientation = q;
            armGoalPosition.position.x = pose.position.x;
            armGoalPosition.position.y = pose.position.y;
            armGoalPosition.position.z = pose.position.z + gripperExt;


            // create linear trajectory
            // cs2cs
            trajectory_msgs::JointTrajectory cs2csTraj;
            youbot_proxy::CSTrajectoryGenerator trajectoryGenCS = this->getCSTrajectoryGenerator(armStartPosition);
            if(!trajectoryGenCS.addPose(armGoalPosition)) {
                ROS_ERROR("Could not create linear grab trajectory");
                return false;
            }
            cs2csTraj = trajectoryGenCS.get();


            ROS_INFO("Generating trajectory to first position (js2js)");
            // extract current joint positions
            sensor_msgs::JointState jointState = this->getJointStates();
            brics_actuator::JointPositions currentJointPositions = youbot_proxy::CSTrajectoryGenerator::jointStateToJointPositions(jointState);

            // extract first point
            brics_actuator::JointPositions firstJointPositions = this->extractFirstPointPositions(cs2csTraj);

            ROS_INFO("Calculating new gripper orientation");
            double j1_current;
            for(int i=0; i<firstJointPositions.positions.size(); i++) {
                if(firstJointPositions.positions[i].joint_uri == "arm_joint_1") {
                    j1_current = firstJointPositions.positions[i].value;
                    break;
                }
            }
            // calculate gripper joint angle
            double j1_correction = j1_current - joint_1_Aty0;
            double joint_limit_correction = - M_PI_2*joint_5_min; //i am a motherfucking genius
            double angle = roll + M_PI + joint_limit_correction + j1_correction; // gripper should be directed forward if roll==0, therefore add PI

            // correct gripper orientation
            for(int i=0; i<firstJointPositions.positions.size(); i++) {
                if(firstJointPositions.positions[i].joint_uri == "arm_joint_5") {
                    firstJointPositions.positions[i].value = angle;
                    break;
                }
            }

            // create trajectory from current joint position to first position of linear trajectory
            // js2js
            trajectory_msgs::JointTrajectory js2jsTraj;
            youbot_proxy::JSTrajectoryGenerator trajectoryGenJs = this->getJSTrajectoryGenerator(currentJointPositions);
            if(!trajectoryGenJs.addPosition(firstJointPositions)) {
                ROS_ERROR("Could not create trajectory to first position");
                return false;
            }
            js2jsTraj = trajectoryGenJs.get();

            // correct orientation of gripper
            ROS_INFO("Correcting gripper orientation");
            this->correctGripperOrientationConst(angle, cs2csTraj);


            ///////////////////////////////////////////////////////////////////////////
            // actual movement phase
            ///////////////////////////////////////////////////////////////////////////
            // open gripper
            this->openGripper();

            // move arm to start pose
            ROS_INFO("Moving arm to first position (js2js)");
            if(!this->move(js2jsTraj)) {
                ROS_ERROR("Could not move arm to start position. Execution of js2js trajectory failed.");
                return false;
            }

            // move arm to grab position
            ROS_INFO("Moving arm to grab position (cs2cs)");
            if(!this->move(cs2csTraj)) {
                ROS_ERROR("Could not move arm to grab position. Execution of cs2cs trajectory failed.");
                return false;
            }

            // close gripper
            this->closeGripper();
            ros::spinOnce();
            ros::Duration(2.0).sleep();

            // move arm back to start position
            ROS_INFO("Moving arm back to start position (cs2cs)");
            TrajectoryGenerator::reverseTrajectory(cs2csTraj);
            if(!this->move(cs2csTraj)) {
                ROS_ERROR("Could not move arm to start position. Execution of cs2cs trajectory failed.");
                return false;
            }

            // move arm to intermediate position
            ROS_INFO("Moving arm to inermediate position");
            brics_actuator::JointPositions interPos = currentJointPositions; // in this case it's the init position
            this->moveArmToJointPosition(interPos);

            // dropping or transporting???


            // finished
            return true;
        }
};

//const std::vector<std::string> Manipulator::JOINT_NAMES = boost::assign::list_of("arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5");
Manipulator::ConstJointNameArrayFiller Manipulator::armJointArrayFiller = Manipulator::ConstJointNameArrayFiller();
Manipulator::ConstGripperNameArrayFiller Manipulator::gripperJointArrayFiller = Manipulator::ConstGripperNameArrayFiller();

class ObjectDetectionListener {
private:
    ros::Subscriber objectPositionSubscriber;

    geometry_msgs::Pose objectPosition;

public:
    ObjectDetectionListener(ros::NodeHandle node, std::string objectPositionTopic) {
        objectPositionSubscriber = node.subscribe<object_recognition::ObjectPosition>(objectPositionTopic, 5, &ObjectDetectionListener::objectPositionCallback, this);
    }

    void objectPositionCallback(const object_recognition::ObjectPositionConstPtr& msg) {
        //if(objectPosition->object_id != "") {
        //    return;
        //}
        geometry_msgs::Pose pose;
        pose.position = msg->pose.position;
        pose.orientation = msg->pose.orientation;
        objectPosition = pose;
    }

    geometry_msgs::Pose getObjectPosition() {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ros::spinOnce();
        return objectPosition;
    }
};

}

//-------------------------------- FUNCTIONS ---------------------------------------------
void testTrajectory(ros::NodeHandle node, youbot_proxy::Manipulator& m) {
    // use of a start postion
    // orientation of the gripper relativ to the base coordinate system:
    // x -> front
    // y -> left
    // z -> top
    //                                                                      x,   y,   z
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0, 1.6, 0.0);
    youbot_proxy::CSTrajectoryGenerator trajectory = m.getCSTrajectoryGenerator(0.27, 0.00, 0.05, q);

    // second point
    if(!trajectory.createAndAddPose(0.27, 0.00, -0.10, q, 0.0)) {
        ROS_ERROR("Unable to create trajectory for 2 CS poses");
        return;
    }

// ATTENTION: first move arm to start position!!!
    ROS_INFO("Extracting start of trajectory and moving arm to start pose...");
    trajectory_msgs::JointTrajectory cs2csTraj = trajectory.get();

    // print first point
    trajectory_msgs::JointTrajectoryPoint p = cs2csTraj.points.back();
    std::stringstream sp;
    for(int i=0; i<p.positions.size(); i++) {
        sp << cs2csTraj.joint_names[i] << ":" << p.positions[i] << "/";
        //if(i == 1) cs2csTraj.points.back().positions[i] = p.positions[i] - 1;
    }
    ROS_INFO_STREAM("First point of cs2cs: " << sp.str());

    brics_actuator::JointPositions targetJointPositions = m.extractFirstPointPositions(cs2csTraj);

    // extract current joint positions
    sensor_msgs::JointState jointState = m.getJointStates();
    brics_actuator::JointPositions currentJointPositions = youbot_proxy::CSTrajectoryGenerator::jointStateToJointPositions(jointState);

    // print current joint positions
    std::stringstream ss1, ss2;
    for(int i=0; i< currentJointPositions.positions.size(); i++) {
        ss1 << currentJointPositions.positions[i].joint_uri << ": " <<
              currentJointPositions.positions[i].value << " " <<
              currentJointPositions.positions[i].unit << "\n";
        ss2 << targetJointPositions.positions[i].joint_uri << ": " <<
              targetJointPositions.positions[i].value << " " <<
              targetJointPositions.positions[i].unit << "\n";
    }
    ROS_INFO_STREAM("Current position: " << ss1.str());
    ROS_INFO_STREAM("Target position: " << ss2.str());

    // create trajectory from current joint position to first position of linear trajectory
    // js2js
    trajectory_msgs::JointTrajectory js2jsTraj;
    youbot_proxy::JSTrajectoryGenerator trajectoryGenJs = m.getJSTrajectoryGenerator(currentJointPositions);
    if(!trajectoryGenJs.addPosition(targetJointPositions)) {
        ROS_ERROR("Could not create trajectory to first position of linear trajectory");
        return;
    }
    js2jsTraj = trajectoryGenJs.get();

    // executing js2js trajectory
    if(!m.move(js2jsTraj)) {
        ROS_ERROR("JS2JS Trajectory executation failed!");
        return;
    }

    ros::Duration(1).sleep();
    ros::spinOnce();
    ROS_INFO("... finished!");
// ATTENTION end

    // check and correct gripper orientation (yaw)

    //double yaw = M_PI_2; // PI/2: gripper is parallel to x
    //m.correctGripperOrientation(yaw, cs2csTraj);


    ROS_INFO("Moving along trajectory");
    if(!m.move(cs2csTraj)) {
        ROS_ERROR("CS2CS Trajectory executation failed!");
    }
}

//----------------------------------- MAIN -----------------------------------------------
int main(int argc, char** argv)
{
    // init node
    ros::init(argc, argv, "cleaner");
    ros::NodeHandle n;

    // create classes
    youbot_proxy::Manipulator m(n);
    youbot_proxy::ObjectDetectionListener objListener(n, "object_position");

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

    // --------------- test grabbing with object recognition ------------------
/*
    // retrieve object position
    geometry_msgs::Pose objectPose = objListener.getObjectPosition();

    // check orientation values
    double roll, pitch, yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(objectPose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    ROS_INFO_STREAM("Object Orientation:\n Roll=" << roll << ", Pitch=" << pitch << ", Yaw=" << yaw);

    /*
    if(pitch != 0) pitch = 0;
    if(yaw != 0) yaw = 0;
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    objectPose.orientation = q;
    *//*

    if(!m.grabObjectAt(objectPose)) {
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
