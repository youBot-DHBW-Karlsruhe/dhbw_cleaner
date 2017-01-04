#include "cleaner_alpha/ObjectFinder.h"

namespace cleaner {

///////////////////////////////////////////////////////////////////////////////
// public methods
///////////////////////////////////////////////////////////////////////////////
ObjectFinder::ObjectFinder(ros::NodeHandle n, std::string scan_topic, std::string base_link):
    baseLink(base_link) {
    ROS_INFO("Entered constructor");
    scanSubscriber = n.subscribe(scan_topic, 10, &ObjectFinder::scanCallback, this);
    markerPublisher = n.advertise<visualization_msgs::Marker>("object_finder_marker", 1);

    initMarker();
}

void ObjectFinder::scanCallback(const sensor_msgs::PointCloud::ConstPtr& pointCloud) {
    nearest = extractNearestPoint(pointCloud->points);
    // execute callback
    // TODO!

    // for testing
    //printPoint(nearest);
    //publishPoint(nearest);
}

/*void registerCallback(boost::function<void (geometry_msgs::Point32 np)> callback) {
  callbackFunction = callback;
}*/

geometry_msgs::Point32 ObjectFinder::nearestPoint() {
  return nearest;
}

void ObjectFinder::publishNearestPoint() {
  publishPoint(nearest);
}

void ObjectFinder::printNearestPoint() {
  printPoint(nearest);
}


///////////////////////////////////////////////////////////////////////////////
// private methods
///////////////////////////////////////////////////////////////////////////////
void ObjectFinder::initMarker() {
  nearestPointMarker.header.frame_id = baseLink;
  nearestPointMarker.header.stamp = ros::Time::now();
  nearestPointMarker.ns = "laser_scans";
  nearestPointMarker.id = 0;
  nearestPointMarker.type = visualization_msgs::Marker::SPHERE;
  nearestPointMarker.action = visualization_msgs::Marker::ADD;
  nearestPointMarker.lifetime = ros::Duration();

  nearestPointMarker.pose.position.x = 0;
  nearestPointMarker.pose.position.y = 0;
  nearestPointMarker.pose.position.z = 0;
  nearestPointMarker.pose.orientation.x = 0.0;
  nearestPointMarker.pose.orientation.y = 0.0;
  nearestPointMarker.pose.orientation.z = 0.0;
  nearestPointMarker.pose.orientation.w = 1.0;

  nearestPointMarker.scale.x = 0.2;
  nearestPointMarker.scale.y = 0.2;
  nearestPointMarker.scale.z = 0.2;

  nearestPointMarker.color.r = 0.0f;
  nearestPointMarker.color.g = 1.0f;
  nearestPointMarker.color.b = 0.0f;
  nearestPointMarker.color.a = 1.0;

}

geometry_msgs::Point32 ObjectFinder::extractNearestPoint(sensor_msgs::PointCloud::_points_type points, double tol) {
    int iNearest = 0;
    for (int i = 1; i <= points.size(); i++) {
        // ignore points with z components
        if(std::abs(points[i].z) > tol) {
            break;
        }
        // ignore points too near to 0
        if(std::abs(points[i].x) < tol && std::abs(points[i].y) < tol) {
            break;
        }

        if(euclDist(points[iNearest]) > euclDist(points[i])) {
            iNearest = i;
        }
    }

    geometry_msgs::Point32 point;
    point.x = points[iNearest].x;
    point.y = points[iNearest].y;
    point.z = points[iNearest].z;
    return point;
}

void ObjectFinder::printPoint(const geometry_msgs::Point32 point) {
  std::stringstream ss;
  ss << "Nearest point: "
        <<   "x=" << point.x
        << ", y=" << point.y
        << ", z=" << point.z;
  ROS_INFO(ss.str().c_str());
}

void ObjectFinder::publishPoint(const geometry_msgs::Point32 point) {
    visualization_msgs::Marker marker = nearestPointMarker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration();

    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z;

    markerPublisher.publish(marker);
}

double ObjectFinder::euclDist(const geometry_msgs::Point32 &point) {
    // z is 0
    return std::sqrt(std::abs(std::pow(point.x, 2) + std::pow(point.y, 2)));
}

}
