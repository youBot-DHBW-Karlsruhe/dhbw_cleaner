#include "cleaner_alpha/ObjectFinder.h"
#include "laser_assembler/AssembleScans.h"

namespace cleaner {

///////////////////////////////////////////////////////////////////////////////
// public methods
///////////////////////////////////////////////////////////////////////////////
ObjectFinder::ObjectFinder(ros::NodeHandle n, std::string cloud_topic, std::string base_link):
    baseLink(base_link)
{
    ros::service::waitForService("assemble_scans");
    nearestService = n.advertiseService("nearest_point", &ObjectFinder::nearestPointServiceHandler, this);
    scanSrvClient = n.serviceClient<laser_assembler::AssembleScans>("assemble_scans");

    cloudPublisher = n.advertise<sensor_msgs::PointCloud>(cloud_topic, 5);
    nearestObjectPublisher = n.advertise<geometry_msgs::Point32>("nearest_object", 5);
    markerPublisher = n.advertise<visualization_msgs::Marker>("object_finder_marker", 5);

    initMarker();
    ROS_INFO("ObjectFinder: Initialized");
}


bool ObjectFinder::nearestPointServiceHandler(object_finder_2d::NearestPoint::Request &req,
                                              object_finder_2d::NearestPoint::Response &res) {
     laser_assembler::AssembleScans scanSrv;
     ros::Time now = ros::Time::now();

     scanSrv.request.begin = now - ros::Duration(1);
     scanSrv.request.end = now;
     if(scanSrvClient.call(scanSrv)) {
         cloud = scanSrv.response.cloud;
         nearest = extractNearestPoint(cloud.points);
         res.point = nearest;
         return true;
     } else {
         ROS_WARN("Service call failed");
         return false;
     }

}

void ObjectFinder::publishIntermediates() {
     sensor_msgs::PointCloud pc = cloud;
     pc.header.frame_id = baseLink;
     pc.header.stamp = ros::Time::now();

     cloudPublisher.publish(pc);
     publishMarker(nearest);
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

  nearestPointMarker.scale.x = 0.15;
  nearestPointMarker.scale.y = 0.15;
  nearestPointMarker.scale.z = 0.15;

  nearestPointMarker.color.r = 1.0f;
  nearestPointMarker.color.g = 1.0f;
  nearestPointMarker.color.b = 0.0f;
  nearestPointMarker.color.a = 1.0;

}

geometry_msgs::Point32 ObjectFinder::extractNearestPoint(sensor_msgs::PointCloud::_points_type& points, double tol) {
    // search first valid point
    geometry_msgs::Point32 point;
    int iNearest = 0;

    while(invalidPointValue(points[iNearest], tol)) {
        if(iNearest >= points.size() - 1) {
            ROS_ERROR("ObjectFinder: No valid points in point cloud from scan data");
            return point;
        }
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
    // "ignore flickering"

    // save point
    point.x = points[iNearest].x;
    point.y = points[iNearest].y;
    point.z = points[iNearest].z;
    return point;
}

bool ObjectFinder::invalidPointValue(const geometry_msgs::Point32 &point, double tol) {
    // ignore points with z components
    if(std::abs(point.z) > tol + 0.06) {
        return true;
    }
    /*
    // ignore robot front
    if(std::abs(point.x) < 0.075 && std::abs(point.y) < 0.20) {
        return true;
    }
    /*

    /*
<property name="base_size_x" value="0.570"/>
<property name="base_size_y" value="0.360"/>
<property name="base_size_z" value="0.100"/>
     */
    // ignore robot front (in case of (0/0) = base_link)
    if(std::abs(point.x) < 0.4+tol && std::abs(point.y) < 0.2+tol) {
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
