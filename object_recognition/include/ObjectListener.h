#ifndef OBJECTKLISTENER_H
#define OBJECTLISTENER_H

#include <ros/ros.h>
#include <find_object_2d/ObjectsStamped.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32MultiArray.h>
#include <object_recognition/ObjectPosition.h>
#include <math.h>
#include <XmlRpcValue.h>

namespace cleaner {

#define OFFSET_Z  0.0
#define OFFSET_Y -0.01
#define OFFSET_X  -0.03

typedef struct {
    std::vector<object_recognition::ObjectPosition> positions;
    std::string objectId;
} ObjectPositions;

typedef struct {
    std::string objectName;
    std::vector<int> images;
    std::vector<int> degrees;
} ObjectPictures;

class ObjectListener {

private:

    std::string baseFrameId_;
    std::string objFramePrefix_;
    tf::TransformListener tfListener_;
    ros::Subscriber subs_;
    ros::Publisher pubAggregated_;
    ros::Publisher pubSingle_;
    std::vector<cleaner::ObjectPositions> positions_;
    std::vector<cleaner::ObjectPictures> objectPicturesParameter_;

    const int aggregedThreshold;

public:

    ObjectListener(ros::NodeHandle nh, std::string base_frame):
        aggregedThreshold(3)
    {
        baseFrameId_ = base_frame;
        objFramePrefix_ = "object";
        objectPicturesParameter_ = loadObjectsFromParameterServer(nh);

        subs_ = nh.subscribe("/objectsStamped", 10, &ObjectListener::objectsDetectedCallback, this);
        pubAggregated_ = nh.advertise<object_recognition::ObjectPosition>("object_position_aggregated", 10);
        pubSingle_ = nh.advertise<object_recognition::ObjectPosition>("object_position_single", 10);
    }

    void objectsDetectedCallback(const find_object_2d::ObjectsStampedConstPtr & msg){

        //ROS_INFO("Callback");
        //convert data
        if(msg->objects.data.size()){
            ROS_INFO("Callback with data");
                    for(unsigned int i=0; i<msg->objects.data.size(); i+=12)
                    {
                        object_recognition::ObjectPosition position;
                        geometry_msgs::Pose coordinates;

                        //get object_id
                        int id = (int)msg->objects.data[i];
                        ros::Time timeStamp = msg->header.stamp;
                        std::stringstream ss;
                        ss << "object_" << id;
                        std::string objectFrameId = ss.str();
                        position.object_id = id;
                        ROS_INFO("object_%d", id);

                        //get saved positions for current object
                        //cleaner::ObjectPositions currentObject = getObjectPositions(objectFrameId);

                        //tf transformation
                        tf::StampedTransform pose;
                        try
                        {
                            // Get transformation from "object_#" frame to target frame "map"
                            // The timestamp matches the one sent over TF

                            tfListener_.lookupTransform(baseFrameId_, objectFrameId, msg->header.stamp, pose);
                        }
                        catch(tf::TransformException & ex)
                        {
                            ROS_WARN("%s",ex.what());
                            continue;
                        }

                        coordinates = assignCoordinates(pose);
                        position.pose = coordinates;
                        position.header.stamp = timeStamp;
                        //currentObject.positions.push_back(position);

                        object_recognition::ObjectPosition singlePosition;
                        singlePosition.object_id = objectFrameId;
                        //singlePosition.object_id = currentObject.objectId;
                        singlePosition.pose = coordinates;
                        singlePosition.header.stamp = timeStamp;
                        publishPosition(pubSingle_, singlePosition);

                        /*if (currentObject.positions.size() >= aggregedThreshold) {
                            object_recognition::ObjectPosition aggregatedPosition = averageAggregation(currentObject);
                            publishPosition(pubAggregated_, aggregatedPosition);
                            deleteObjectPositions(currentObject);
                        }
                        writeBackObjectPositions(currentObject);*/

                    }
        }

    }

    void publishPosition(ros::Publisher pub, object_recognition::ObjectPosition position) {
        int id = getPictureId(position);
        normalizeGraspingDegree(id, position.pose.orientation);
        //rollDegree = roll * 180 / M_PI;
        /*if(position.object_id.find("quer") != std::string::npos){ //think of a useful condition here
            ROS_INFO("Objekt quer");
            changeGraspingDegree(position.pose.orientation);
        }else{
            ROS_INFO("Objekt laengs");
            normalizeGraspingDegree(position.pose.orientation);
        }*/
        pub.publish(position);
    }

    int getPictureId(object_recognition::ObjectPosition position){
        std::vector<std::string> object_id = split(position.object_id,'_');
        return std::stoi(object_id[1]);
    }

    cleaner::ObjectPositions getObjectPositions(std::string objectId){
        for(int i = 0; i < positions_.size(); i++){
            if(positions_.at(i).objectId == objectId){
                return positions_.at(i);
            }
        }
        cleaner::ObjectPositions newObject;
        newObject.objectId = objectId;
        positions_.push_back(newObject);
        return newObject;
    }

    void writeBackObjectPositions(cleaner::ObjectPositions currentObject){
        for(int i = 0; i < positions_.size(); i++){
            if(positions_.at(i).objectId == currentObject.objectId){
                positions_.at(i) = currentObject;
                return;
            }
        }
    }

    void deleteObjectPositions(cleaner::ObjectPositions &currentObject){
        currentObject.positions.clear();
    }

    geometry_msgs::Pose assignCoordinates(tf::StampedTransform pose){
        geometry_msgs::Pose coordinates;
        coordinates.position.x = pose.getOrigin().x() + OFFSET_X;
        coordinates.position.y = pose.getOrigin().y() + OFFSET_Y;
        coordinates.position.z = pose.getOrigin().z() + OFFSET_Z;
        coordinates.orientation.x = pose.getRotation().x();
        coordinates.orientation.y = pose.getRotation().y();
        coordinates.orientation.z = pose.getRotation().z();
        coordinates.orientation.w = pose.getRotation().w();
        return coordinates;
    }

    geometry_msgs::Pose addBiasedCoordinates(geometry_msgs::Pose finalCoordinates, geometry_msgs::Pose addedCoordinates, int div){
        finalCoordinates.position.x += addedCoordinates.position.x / div;
        finalCoordinates.position.y += addedCoordinates.position.y / div;
        finalCoordinates.position.z += addedCoordinates.position.z / div;
        finalCoordinates.orientation.x += addedCoordinates.orientation.x / div;
        finalCoordinates.orientation.y += addedCoordinates.orientation.y / div;
        finalCoordinates.orientation.z += addedCoordinates.orientation.z / div;
        finalCoordinates.orientation.w += addedCoordinates.orientation.w / div;
        return finalCoordinates;
    }

    void normalizeGraspingDegree(int pictureId, geometry_msgs::Quaternion &msgQuat){
        double roll, pitch, yaw;
        tf::Quaternion tfQuat;
        tf::quaternionMsgToTF(msgQuat,tfQuat);
        tf::Matrix3x3(tfQuat).getRPY(roll, pitch, yaw);

        double additionialRoll = getDegree(pictureId) * M_PI / 180;
        roll += additionialRoll;
        pitch = 0;
        yaw = 0;

	ROS_INFO("Roll before normalization: %f", roll * 180 / M_PI);
        if(roll > M_PI_2){
            roll = -M_PI_2 + (roll-M_PI_2);
        }else
            if(roll < -M_PI_2){
            roll = M_PI_2 + (roll+M_PI_2);
        }
        ROS_INFO("Roll after normalization: %f", roll * 180 / M_PI);

        tfQuat.setRPY(roll, pitch, yaw);
        tf::quaternionTFToMsg(tfQuat, msgQuat);
    }

    object_recognition::ObjectPosition averageAggregation(cleaner::ObjectPositions currentObject) {
        object_recognition::ObjectPosition aggregatedPosition;
        aggregatedPosition.object_id = currentObject.objectId;
        for (int i = 0; i < currentObject.positions.size(); i++){
            if(i = 0){
                aggregatedPosition.header.stamp = currentObject.positions.at(i).header.stamp;
            }
            aggregatedPosition.pose = addBiasedCoordinates(aggregatedPosition.pose, currentObject.positions.at(i).pose, currentObject.positions.size());
        }
        return aggregatedPosition;
    }

    std::vector<cleaner::ObjectPictures> loadObjectsFromParameterServer(ros::NodeHandle nh){
        std::vector<cleaner::ObjectPictures> objectsPicturesList;

        XmlRpc::XmlRpcValue objects;
        nh.getParam("object_config/objects", objects);
        ROS_ASSERT(objects.getType() == XmlRpc::XmlRpcValue::TypeStruct);

        for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = objects.begin(); it != objects.end(); ++it) {
            cleaner::ObjectPictures objectPictures;
            objectPictures.objectName = (std::string)(it->first);

            XmlRpc::XmlRpcValue object;
            std::stringstream objectName;
            objectName << "object_config/objects/" << (std::string)(it->first);
            nh.getParam(objectName.str().c_str(), object);
            ROS_ASSERT(object.getType() == XmlRpc::XmlRpcValue::TypeString);

            std::vector<std::string> images = split(object, '-');
            for(int i = 1; i < images.size(); i++){
                std::vector<std::string> image = split(images[i], ':');
                objectPictures.images.push_back(std::stoi(image[0]));
                objectPictures.degrees.push_back(std::stoi(image[1]));
            }

            objectsPicturesList.push_back(objectPictures);
        }
        return objectsPicturesList;
    }

    int getDegree(int objectId){
        for(int i=0; i < objectPicturesParameter_.size(); i++){
            for(int j=0; j < objectPicturesParameter_[i].images.size(); j++){
                if(objectPicturesParameter_[i].images[j] == objectId){
                    return objectPicturesParameter_[i].degrees[j];
                }
            }
        }
    }

    std::vector<std::string> split(const std::string &s, char delim) {
        std::vector<std::string> elems;
        split(s, delim, std::back_inserter(elems));
        return elems;
    }

    template<typename Out>
    void split(const std::string &s, char delim, Out result) {
        std::stringstream ss;
        ss.str(s);
        std::string item;
        while (std::getline(ss, item, delim)) {
            *(result++) = item;
        }
    }

};

}

#endif
