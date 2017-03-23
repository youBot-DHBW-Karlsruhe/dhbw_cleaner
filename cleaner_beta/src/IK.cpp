#include <ros/ros.h>
#include <youbot_arm_kinematics/inverse_kinematics.h>
#include <youbot_arm_kinematics/configuration_comparator.h>
#include <youbot_arm_kinematics/logger.h>
#include <urdf/model.h>
#include <boost/shared_ptr.hpp>
#include <tf_conversions/tf_kdl.h>

#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

/*
namespace youbot_arm_kinematics {

Logger Logger::null;

Logger::Logger(){}

Logger::~Logger(){}

void Logger::write(const std::string &msg, const char *file, int line)
{
    std::cout << msg << std::endl;
}

}
*/

class IK {

public:

    IK(ros::NodeHandle n, std::string robot_description, std::string base_frame, std::string tip_frame, bool isDebug):
        NUM_JOINTS(5),
        debug(isDebug)
    {

        // get joint descriptions from robot model
        urdf::Model robot_model;
        if (!robot_model.initParam(robot_description)) {
            ROS_ERROR("Could not load robot description");
        }

        std::vector<double> lower_limits;
        std::vector<double> upper_limits;
        if (!extractKinematicData(robot_model, base_frame, tip_frame, lower_limits, upper_limits)) {
            ROS_ERROR("Could not load joint limits");
        }

        // Validate that the correct number of joints has been extracted
        if (joint_names.size() != NUM_JOINTS) {
            ROS_ERROR("Wrong number of joints");
        }

        // debug output
        if(debug) {
            std::stringstream ss;
            for(int i=0; i<joint_names.size(); i++) {
                ss << "        " << joint_names[i]
                   << "(" << lower_limits[i] << ", " << upper_limits[i] << ")"
                   << std::endl;
            }
            ROS_INFO_STREAM("Joints: " << std::endl << ss.str());
        }

        ik.reset(new youbot_arm_kinematics::InverseKinematics(lower_limits, upper_limits));

        if(debug) ROS_INFO("IK solver initialized");
    }

    IK(ros::NodeHandle n, std::string robot_description, std::string base_frame, std::string tip_frame):
        NUM_JOINTS(5)
    {
        IK(n, robot_description, base_frame, tip_frame, false);
    }

    virtual ~IK(){}

    bool solveClosestIK(const geometry_msgs::Pose &ik_pose,
            const std::vector<double> &ik_reference_state,
            std::vector<double> &solution) const
    {
        // Check if the initialize function has already been called successfully
        if (!ik) return false;

        // Validate that there is one seed value per joint
        if (ik_reference_state.size() != NUM_JOINTS) return false;


        // Convert the ROS pose to a KDL frame
        KDL::Frame frame;
        tf::poseMsgToKDL(ik_pose, frame);

        // Convert the seed array to a KDL joint array
        KDL::JntArray seed = configurationStdToKdl(ik_reference_state);

        // Calculate the inverse position kinematics
        std::vector<KDL::JntArray> kdl_solutions;
        int res = ik->CartToJnt(seed, frame, kdl_solutions);

        if(debug) ROS_INFO_STREAM("found " << kdl_solutions.size() << " solutions");

        if (res <= 0) {
            //NO_IK_SOLUTION;
            return false;
        }

        // Convert the found solution from a KDL joint array to a vector of doubles
        std::vector<std::vector<double> > solutions(kdl_solutions.size());
        for (std::size_t i = 0; i < kdl_solutions.size(); i++) {
            solutions[i] = configurationKdlToStd(kdl_solutions[i]);
        }

        // Sort the solutions based on the distance to the seed state and return the
        // closest solution (i.e. the first one after sorting).
        ConfigurationComparator<double> comp(ik_reference_state);
        std::sort(solutions.begin(), solutions.end(), comp);
        solution = solutions[0];

        // SUCCESS;
        return true;
    }

    void setDebug(bool debug) {
        this->debug = debug;
    }

    /**
     * Creates a geometry_msgs::Pose from x, y, z-Values and a quaternion.
     *
     * @param x the x coordinate in the base_frame of the manipulator
     * @param y the y coordinate in the base_frame of the manipulator
     * @param z the z coordinate in the base_frame of the manipulator
     * @param gripperOrientation the orientation of the gripper as a geometry_msgs::Quaternion
     *
     * @return The composed pose
     */
    static const geometry_msgs::Pose createPose(double x, double y, double z, const geometry_msgs::Quaternion gripperOrientation) {
        geometry_msgs::Pose kp;
        kp.position.x = x;
        kp.position.y = y;
        kp.position.z = z;
        kp.orientation = gripperOrientation;
        return kp;
    }


private:

    bool extractKinematicData(const urdf::Model& robot_model,
            const std::string& base_frame,
            const std::string& tip_frame,
            std::vector<double>& lower_limits,
            std::vector<double>& upper_limits)
    {
        boost::shared_ptr<urdf::Link> link = boost::const_pointer_cast<urdf::Link>(robot_model.getLink(tip_frame));

        while ((link) && (link->name != base_frame)) {
            link_names.push_back(link->name);
            boost::shared_ptr<urdf::Joint> joint = link->parent_joint;

            // Don't consider invalid, unknown or fixed joints
            if ((!joint) || (joint->type == urdf::Joint::UNKNOWN)
                    || (joint->type == urdf::Joint::FIXED)) {
                // Continue with the next link in the kinematic chain
                link = link->getParent();

                continue;
            }

            joint_names.push_back(joint->name);

            // Extract the joint limits
            if (joint->type != urdf::Joint::CONTINUOUS) {
                if (joint->safety) {
                    lower_limits.push_back(joint->safety->soft_lower_limit);
                    upper_limits.push_back(joint->safety->soft_upper_limit);
                } else {
                    lower_limits.push_back(joint->limits->lower);
                    upper_limits.push_back(joint->limits->upper);
                }
            } else {
                lower_limits.push_back(-M_PI);
                upper_limits.push_back( M_PI);
            }

            // Continue with the next link in the kinematic chain
            link = link->getParent();
        }

        // The kinematic chain ended and the base frame was not found
        if (!link) return false;

        // The data has been extracted from the tip to the base, but it is required
        // the other way round
        std::reverse(link_names.begin(), link_names.end());
        std::reverse(joint_names.begin(), joint_names.end());
        std::reverse(lower_limits.begin(), lower_limits.end());
        std::reverse(upper_limits.begin(), upper_limits.end());

        return true;
    }

    /**
     * Convert a joint configuration vector, in which each value is
     * represented as a double, to a KDL joint array. The opposite
     * conversion is implemented in the {@see configurationKdlToStd}
     * function.
     *
     * @param v The input configuration.
     *
     * @return The input configuration converted to a joint array.
     */
    KDL::JntArray configurationStdToKdl(const std::vector<double>& v) const
    {
        KDL::JntArray jnt(v.size());

        for (std::size_t i = 0; i < v.size(); i++) {
            jnt(i) = v[i];
        }

        return jnt;
    }

    /**
     * Convert a KDL joint array to a joint configuration vector, in which
     * each value is represented as a double. The opposite conversion is
     * implemented in the {@see configurationStdToKdl} function.
     *
     * @param jnt The input joint configuration.
     *
     * @return The converted joint configuration vector.
     */
    std::vector<double> configurationKdlToStd(const KDL::JntArray& jnt) const
    {
        std::vector<double> v(jnt.rows());

        for (int i = 0; i < jnt.rows(); i++) {
            v[i] = jnt(i);
        }

        return v;
    }

    /**
     * The number of joints for which the inverse kinematics can solve.
     */
    const std::size_t NUM_JOINTS;

    /**
     * The joints on which the kinematics plugin is working.
     */
    std::vector<std::string> joint_names;

    /**
     * The links in the kinematic chain between the base frame and the tip
     * frame (as provided to the @see KinematicsPlugin::initialize
     * function).
     */
    std::vector<std::string> link_names;

    /**
     * The inverse kinematics solver.
     */
    boost::shared_ptr<youbot_arm_kinematics::InverseKinematics> ik;

    bool debug;
};


//----------------------------------- MAIN -----------------------------------------------
int main(int argc, char** argv)
{
  // init node
  ros::init(argc, argv, "cleaner");
  ros::NodeHandle n;

  IK ik(n, "/robot_description", "arm_link_0", "arm_link_5", false);

  // orientation of the gripper relativ to the base coordinate system:
  // x -> front
  // y -> left
  // z -> top
  // angle in radiant
  double rx = 0.0;
  double ry = 3/4 * M_PI;
  double rz = 0.0;
  //                                                                      x,   y,   z
  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw( rx,  ry,  rz);
  //geometry_msgs::Quaternion q;
  geometry_msgs::Pose p = ik.createPose(0.34, 0.00, 0.30, q);
  std::vector<double> joints(5);
  joints[0] = 1.0;
  joints[1] = 1.0;
  joints[2] = 1.0;
  joints[3] = -1.0;
  joints[4] = 1.0;
  std::vector<double> solution;

/*
  std::cout << "Positions for Orientation: Roll=" << rx << ", Pitch=" << ry << ", Yaw=" << rz << std::endl;
  std::cout << "----------------------------------------------------------------------" << std::endl << std::endl;
  for(double x=-0.7; x<=0.7; x+=0.01) {
      for(double y=-0.7; y<=0.7; y+=0.01) {
          for(double z=-1.0; z<=1; z+=0.01) {
              p = ik.createPose(x, y, z, q);
              int res = ik.solveClosestIK(p, joints, solution);
              if(res != 0)
                  std::cout << x << "/" << y << "/" << z << std::endl;
          }
          if(!ros::ok()) {
              return 1;
          }
      }
  }/**/


  bool res = ik.solveClosestIK(p, joints, solution);

  if(!res) {
      ROS_ERROR("No solution found!");
  }

  if(solution.size() != 5) {
      ROS_ERROR("Solution does not provide a value for each joint");
      return -1;
  }
  ROS_INFO_STREAM("Solved IK: " << solution[0] << " "
                                << solution[1] << " "
                                << solution[2] << " "
                                << solution[3] << " "
                                << solution[4] << " ");/**/

  return 0;
}
