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
    std::stringstream ss;
    ss << "nearest_object_" <<  scan_topic.erase(0, 1);
    nearestObjectPublisher = n.advertise<geometry_msgs::Point32>(ss.str().c_str(), 5);

    initMarker();
}

void ObjectFinder::scanCallback(const sensor_msgs::PointCloud::ConstPtr& pointCloud) {
    geometry_msgs::Point32 nearest = extractNearestPoint(pointCloud->points);
    if(std::abs(nearest.x) < 0.01 && std::abs(nearest.y) < 0.01 && std::abs(nearest.z) < 0.01) {
        return;
    }
    nearestObjectPublisher.publish(nearest);

    // debugging infos
    publishMarker(nearest);
    //printPoint(nearest);
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

  nearestPointMarker.scale.x = 0.15;
  nearestPointMarker.scale.y = 0.15;
  nearestPointMarker.scale.z = 0.15;

  nearestPointMarker.color.r = 1.0f;
  nearestPointMarker.color.g = 1.0f;
  nearestPointMarker.color.b = 0.0f;
  nearestPointMarker.color.a = 1.0;

}

geometry_msgs::Point32 ObjectFinder::extractNearestPoint(sensor_msgs::PointCloud::_points_type points, double tol) {
    // search first valid point
    int iNearest = 0;
    while(invalidPointValue(points[iNearest], tol)) {
        iNearest++;
    }
    // check for nearer ones
    for (int i = iNearest+1; i <= points.size(); i++) {
        // ignore invalid points
        if(invalidPointValue(points[i], tol)) {
            break;
        }

        if(euclDist(points[iNearest]) > euclDist(points[i])) {
            iNearest = i;
        }
    }

    // TODO: only count points that are there over a specific period of time
    // "ignore flicker"

    // save point
    geometry_msgs::Point32 point;
    point.x = points[iNearest].x;
    point.y = points[iNearest].y;
    point.z = points[iNearest].z;
    return point;
}

bool ObjectFinder::invalidPointValue(const geometry_msgs::Point32 &point, double tol) {
    // ignore points with z components
    if(std::abs(point.z) > tol) {
        return true;
    }
    // ignore robot front
    if(std::abs(point.x) < 0.075 && std::abs(point.y) < 0.20) {
        return true;
    }

    return false;
}

void ObjectFinder::printPoint(const geometry_msgs::Point32 point) {
  std::stringstream ss;
  ss << "Nearest point: "
        <<   "x=" << point.x
        << ", y=" << point.y
        << ", z=" << point.z;
  ROS_INFO(ss.str().c_str());
}

void ObjectFinder::publishMarker(const geometry_msgs::Point32 point) {
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
