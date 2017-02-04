#include "ros/ros.h"
#include "laser_assembler/AssembleScans.h"
#include "sensor_msgs/PointCloud.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "laser_assembler_test");
    ros::NodeHandle n;
    ros::service::waitForService("assemble_scans");
    ros::ServiceClient client = n.serviceClient<laser_assembler::AssembleScans>("assemble_scans");
    ros::Publisher scanPublisher = n.advertise<sensor_msgs::PointCloud>("assembled_cloud", 1);

    laser_assembler::AssembleScans srv;

    while(ros::ok()) {
        srv.request.begin = ros::Time::now() - ros::Duration(0.5);
        srv.request.end = ros::Time::now();
        if(client.call(srv)) {
            scanPublisher.publish(srv.response.cloud);
        } else {
            ROS_WARN("Service call failed");
        }
        ros::spinOnce();
    }

    return 0;
}
