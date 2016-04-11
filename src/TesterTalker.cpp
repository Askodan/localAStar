#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_msgs/TFMessage.h>

#include <sstream>

void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scanMsg)
{
    int scanNumber = scanMsg->scan_time / scanMsg->time_increment;
    printf("Scan number: %d \n", scanNumber);


}

void TfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    printf("size of ranges: %d\n", sizeof(msg->transforms));
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "TesterTalker");

    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("Gadam", 1000);
    ros::Subscriber subScan = n.subscribe("/batman/scan", 100, ScanCallback);
    ros::Subscriber tfScan = n.subscribe("/tf", 100, TfCallback);

    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "no, bez jaj " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
  }

    return 0;
}
