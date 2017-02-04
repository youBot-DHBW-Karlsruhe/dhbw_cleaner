#ifndef LASER2CLOUD_H
#define LASER2CLOUD_H

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

namespace cleaner {

class LaserScanToPointCloud {
public:
	typedef boost::function<void (const sensor_msgs::PointCloud&)> t_callback;
	typedef std::map<std::string, t_callback> t_callback_map;

	LaserScanToPointCloud(ros::NodeHandle n, std::string scan_topic, std::string cloud_topic, std::string base_link);

	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scanIn);

	void registerCallback(std::string name, t_callback callback);
	
	void deregisterCallback(std::string name);

private: 
  
	std::string baseLink;
	tf::TransformListener tfListener;
	t_callback_map callbacks;
	message_filters::Subscriber<sensor_msgs::LaserScan> laserSubscriber;
	tf::MessageFilter<sensor_msgs::LaserScan> laserFilter;
	ros::Publisher scanPublisher;


};

}

#endif // LASER2CLOUD_H
