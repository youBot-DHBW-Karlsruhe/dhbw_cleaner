#include "cleaner_alpha/YoubotBase.h"


namespace youbot_proxy {

///////////////////////////////////////////////////////////////////////////////
// public methods
///////////////////////////////////////////////////////////////////////////////
YoubotBase::YoubotBase(ros::NodeHandle& node, std::string base_vel_topic):
    pointSeconds(DEFAULT_POINT_SECONDS),
    speed(DEFAULT_SPEED)
{
    initialize(node, base_vel_topic);
}

YoubotBase::YoubotBase(ros::NodeHandle& node, std::string base_vel_topic, double movementSecondsForEachPoint):
    pointSeconds(std::abs(movementSecondsForEachPoint)),
    speed(DEFAULT_SPEED)
{
    initialize(node, base_vel_topic);
}

YoubotBase::YoubotBase(ros::NodeHandle& node, std::string base_vel_topic, double movementSecondsForEachPoint, double baseMovementSpeed):
    pointSeconds(std::abs(movementSecondsForEachPoint)),
    speed(std::abs(baseMovementSpeed))
{
    initialize(node, base_vel_topic);
}

YoubotBase::~YoubotBase(){}

void YoubotBase::move(const Direction *direction, double distanceInMeters) {
    geometry_msgs::Twist moveMsg;

    if(speed <= TOLERANCE/10.0) {
        return;
    }
    if(distanceInMeters <= TOLERANCE) {
        return;
    }

    // diagonal case: calculate distance in x,y and then the time
    double duration = 0;
    if(direction->isDiagonal()) {
        duration = (1/speed) * ((distanceInMeters/2) * std::sqrt(2));
    } else {
        duration = (1/speed) * distanceInMeters;
    }

    switch(direction->type) {
    case LEFT_FORWARD:
        moveMsg.linear.x = speed;
        moveMsg.linear.y = speed;
        break;
    case FORWARD:
        moveMsg.linear.x = speed;
        break;
    case RIGHT_FORWARD:
        moveMsg.linear.x = speed;
        moveMsg.linear.y = -speed;
        break;
    case LEFT:
        moveMsg.linear.y = speed;
        break;
    case RIGHT:
        moveMsg.linear.y = -speed;
        break;
    case LEFT_BACKWARD:
        moveMsg.linear.x = -speed;
        moveMsg.linear.y = speed;
        break;
    case BACKWARD:
        moveMsg.linear.x = -speed;
        break;
    case RIGHT_BACKWARD:
        moveMsg.linear.x = -speed;
        moveMsg.linear.y = -speed;
        break;
    default:
        return;
    }

    this->pubBase.publish(moveMsg);
    ros::Duration(duration).sleep();
    this->pubBase.publish(this->stopMsg);
}

void YoubotBase::move(double x, double y) {
    geometry_msgs::Twist moveMsg;

    if(speed <= TOLERANCE/10.0) {
        return;
    }
    if(y <= TOLERANCE && x <= TOLERANCE) {
        return;
    }

    // calculate different speeds for x and y
    double s, t, vx, vy;
    s  = std::sqrt(std::pow(x, 2) + std::pow(y, 2)); // [s] = [x] = [y] = meter
    t  = s / speed;                                  // [t] = second
    vx = x / t;                                      // [vx] = meter/second
    vy = y / t;                                      // [vy] = meter/second
                                                     // [speed] = meter/second

    moveMsg.linear.x = vx;
    moveMsg.linear.y = vy;
    moveMsg.linear.z =  0;

    this->pubBase.publish(moveMsg);
    ros::Duration(t).sleep();
    this->pubBase.publish(this->stopMsg);
}

void YoubotBase::turnDeg(double angleInDeg) {
    this->turnRad(DEG_TO_RAD(angleInDeg));
}

void YoubotBase::turnRad(double angleInRad) {
    //double speed = 0.5;

    if(std::abs(angleInRad) <= TOLERANCE) {
        return;
    }
    if(speed <= TOLERANCE/10.0) {
        return;
    }
    double duration = (1/speed) * std::abs(angleInRad);
    geometry_msgs::Twist turnMsg;
    turnMsg.angular.z = ((angleInRad < 0)? -1 : 1) * speed;

    this->pubBase.publish(turnMsg);
    ros::Duration(duration).sleep();
    this->pubBase.publish(this->stopMsg);
}

///////////////////////////////////////////////////////////////////////////////
// private methods
///////////////////////////////////////////////////////////////////////////////
void YoubotBase::initialize(ros::NodeHandle& node, std::string base_vel_topic) {
    // create stop message
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    this->stopMsg = msg;

    // create publisher
    ros::Publisher basePublisher = node.advertise<geometry_msgs::Twist>(base_vel_topic, 1);

    // check status of members
    if(!basePublisher) {
        ROS_ERROR("Initialization failed: publishers could not be created");
        exit(1);
// TODO: throw error instead killing the process
    }
    this->pubBase = basePublisher;
}

}
