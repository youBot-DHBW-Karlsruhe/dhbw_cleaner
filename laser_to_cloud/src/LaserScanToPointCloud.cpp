#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

class LaserScanToPointCloud {
public:
    typedef boost::function<void (const sensor_msgs::PointCloud&)> t_callback;

private: 
  
  std::string baseLink;
  tf::TransformListener tfListener;
  message_filters::Subscriber<sensor_msgs::LaserScan> laserSubscriber;
  tf::MessageFilter<sensor_msgs::LaserScan> laserFilter;
  ros::Publisher scanPublisher;
  std::vector<t_callback> callbacks;

public:

  LaserScanToPointCloud(ros::NodeHandle n, std::string scan_topic, std::string cloud_topic, std::string base_link):
    baseLink(base_link),
    tfListener(),
    laserSubscriber(n, scan_topic, 10),
    laserFilter(laserSubscriber, tfListener, base_link, 10)
  {
    ROS_INFO("Entered constructor");
    laserFilter.registerCallback(boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laserFilter.setTolerance(ros::Duration(0.01));
    scanPublisher = n.advertise<sensor_msgs::PointCloud>(cloud_topic,1);
    ROS_INFO("LaserScanToPointCloud: initialized all objects");
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scanIn)
  {
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud cloud;
    ros::Time timeSource = scanIn->header.stamp + ros::Duration().fromSec(scanIn->ranges.size() * scanIn->time_increment);
    ros::Duration timeout = ros::Duration(1.0);

    try {
        // quit if no transform is available
        if(!tfListener.waitForTransform(scanIn->header.frame_id, baseLink, timeSource, timeout))
        {
            return;
        }
        projector.transformLaserScanToPointCloud(baseLink, *scanIn, cloud, tfListener);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        ROS_ERROR(e.what());
        return;
    }

    // publish point cloud on new topic
    scanPublisher.publish(cloud);

    // forward to callbacks
    for(int i = 0; i < callbacks.size(); i++) {
        callbacks.at(i)(cloud);
    }

  }

  void registerCallback(t_callback callback) {
      callbacks.push_back(callback);
  }

  void deregisterCallback(t_callback callback) {
      //TODO!
  }
};


//----------------------------------- MAIN -----------------------------------------------
void callback(const sensor_msgs::PointCloud & cloud) {
    ROS_INFO("received callback");
}

int main(int argc, char** argv)
{
  // init node
  ros::init(argc, argv, "scan_to_cloud");
  ros::NodeHandle n;

  // retrieve parameters
  std::string from_topic, to_topic, tf_base_link;
  ros::param::param<std::string>("laser_scan_topic", from_topic, "/laser_scan");
  ros::param::param<std::string>("transform_cloud_topic", to_topic, "/laser_point_cloud");
  ros::param::param<std::string>("base_link", tf_base_link, "base_link");

  ROS_INFO("Using %s as laser_scan_topic", from_topic.c_str());
  ROS_INFO("Using %s as transform_cloud_topic", to_topic.c_str());
  ROS_INFO("Using %s as base_link", tf_base_link.c_str());

  LaserScanToPointCloud lstopc(n, from_topic, to_topic, tf_base_link);
  ROS_INFO("Transformation running");
  lstopc.registerCallback(callback);

  ros::spin();
  
  return 0;
}
