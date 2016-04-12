#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_listener.h>
#include <sstream>


///global variables
geometry_msgs::Pose goalPosition;
geometry_msgs::Pose currentPosition;
const double tf::Transformer::DEFAULT_CACHE_TIME = 10.0;

///function dectarations
void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scanMsg);
void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
void UpdateCurrentPosition(geometry_msgs::Pose &newPose, tf::TransformListener* listener);

///main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "TesterTalker");

    ros::NodeHandle n;
    ros::Subscriber subScan = n.subscribe("/batman/scan", 100, ScanCallback);
    ros::Subscriber goalSub = n.subscribe("/move_base_simple/goal", 20, GoalCallback);
    tf::TransformListener transformListener;
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        //UpdateCurrentPosition(currentPosition, &transformListener);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}


void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scanMsg)
{
    int scanNumber = scanMsg->scan_time / scanMsg->time_increment;
}

void TfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{

}

void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    ///copy the given goal to global variable
    memcpy(&goalPosition, &(goalMsg->pose), sizeof(goalPosition));

    float x = goalMsg->pose.position.x;
    float y = goalMsg->pose.position.y;
    float z = goalMsg->pose.position.z;
    printf("goal: x, y, z: %f, %f, %f\n", x, y, z);
}

void UpdateCurrentPosition(geometry_msgs::Pose &newPose, tf::TransformListener* listener)
{
    tf::StampedTransform transform;

    try
    {
        listener->lookupTransform("/tf", "batman/base_laser_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    memcpy(&(newPose.position), transform.getOrigin(), sizeof(newPose.position));
    memcpy(&(newPose.orientation), transform.getRotation(), sizeof(newPose.orientation));

    printf("New pose: %f, %f, %f\n", newPose.position.x, newPose.position.y, newPose.position.z);
}
