#include "cleaner_alpha/LaserScanToPointCloud.h"
#include "laser_geometry/laser_geometry.h"
#include "algorithm"

namespace cleaner {

///////////////////////////////////////////////////////////////////////////////
// public methods
///////////////////////////////////////////////////////////////////////////////
LaserScanToPointCloud::LaserScanToPointCloud(ros::NodeHandle n, std::string scan_topic, std::string cloud_topic, std::string base_link):
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

void LaserScanToPointCloud::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scanIn) {
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud cloud;
    ros::Time timeSource = scanIn->header.stamp + ros::Duration().fromSec(scanIn->ranges.size() * scanIn->time_increment);
    ros::Duration timeout = ros::Duration(1.0);

    try {
        // quit if no transform is available
        if(!tfListener.waitForTransform(scanIn->header.frame_id, baseLink, timeSource, timeout)) {
            return;
        }
        projector.transformLaserScanToPointCloud(baseLink, *scanIn, cloud, tfListener);
    } catch (tf::TransformException& e) {
        std::cout << e.what();
        ROS_ERROR(e.what());
        return;
    }

    // publish point cloud on new topic
    scanPublisher.publish(cloud);

    // forward to callbacks
    t_callback_map::iterator iter;
    for(iter=callbacks.begin(); iter!=callbacks.end(); ++iter) {
        iter->second(cloud);
    }

}

void LaserScanToPointCloud::registerCallback(std::string name, t_callback callback) {
      callbacks[name] = callback;
}

void LaserScanToPointCloud::deregisterCallback(std::string name) {
      callbacks.erase(name);
}

///////////////////////////////////////////////////////////////////////////////
// private methods
///////////////////////////////////////////////////////////////////////////////

}
