#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_listener.h>
#include <sstream>
#include <math.h>

///global variables
geometry_msgs::Pose goalPosition;
geometry_msgs::Pose currentPosition;
const double tf::Transformer::DEFAULT_CACHE_TIME = 10.0;
laser_geometry::LaserProjection projector;
ros::Publisher pointPub;
ros::Publisher markerViz;
float robotWidth = 0.5;//ważny parametr grubości naszego ulubieńca najlepiej podawać wraz z zapasem odległości od przeszkód
bool Ruszaj_Batmobilu = false;

///function dectarations
void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scanMsg);
void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
void UpdateCurrentPosition(geometry_msgs::Pose &newPose, tf::TransformListener* listener);
bool isPathFree(geometry_msgs::Pose there, sensor_msgs::PointCloud* obstacles);
void makeMarker(geometry_msgs::PoseStamped pos, float r, float g, float b);
void makeMarker(float x, float y, float r, float g, float b);


///main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "LocalPlanner");

    ros::NodeHandle n;
    ros::Subscriber subScan = n.subscribe("/batman/scan", 100, ScanCallback);
    ros::Subscriber goalSub = n.subscribe("/move_base_simple/goal", 20, GoalCallback);
    pointPub = n.advertise<geometry_msgs::PoseStamped>("/path/chwilowy", 1000);
    markerViz = n.advertise<visualization_msgs::Marker>("/markers", 1000);
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
    sensor_msgs::PointCloud cloud;
    projector.projectLaser(*scanMsg, cloud);
    //to nie jest miejsce na wywołanie tej funkcji, ale do testu się nadaje
    isPathFree(goalPosition, &cloud);
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
std::vector <geometry_msgs::Pose> Find_Path(){
    std::vector <geometry_msgs::Pose> somepath;
    return somepath;
}
//funkcja posiada dwa założenia:
//  a) chmura i punkt docelowy są w układzie robota
//  b) cel jest dokładnie przed robotem
bool isPathFree(geometry_msgs::Pose there, sensor_msgs::PointCloud* obstacles){
    //przy spełnionych założeniach x jest wystarczający
    float distance = std::sqrt(there.position.x*there.position.x+there.position.y*there.position.y)+robotWidth/2;

    bool isFree = true;
    //int k = 0;
    //int l = 0;
    //ogólnie oś x - do przodu, oś y - w bok(jeśli wierzyć rvizowi, to w lewo), oś z do góry
    for(int i =0;i<obstacles->points.capacity();i++){
        //sprawdza, czy punkt nie znajduje się za robotem
        if(obstacles->points[i].x<0){
           //nic nie robi, jak jest za robotem - zakres skanera zdaje się być większy niż 180 stopni
        }else{
            if(obstacles->points[i].x<distance && std::abs(obstacles->points[i].y)<robotWidth/2){
                isFree = false;
                makeMarker(obstacles->points[i].x, obstacles->points[i].y, 1,1,1);
            }
        }
        /*
        //znajduje punkt najbardziej z przodu robota
        if(i>0){
            if(obstacles->points[i-1].y*obstacles->points[i].y<0){
                k=i;
                l++;
            }
        }
        */
        //kończy pętle, gdy dojdzie do pustych punktów
        if(obstacles->points[i].x==0.0 && obstacles->points[i].y==0.0){
            break;
        }
    }
    //std::cout<<k<<" k"<<obstacles->points[k]<<" k+1 "<<obstacles->points[k+1]<<" k-1 "<<obstacles->points[k-1]<<std::endl;
    //std::cout<<l<<std::endl;

    /*
    //publikuje piękną strzałkę w rvizie :D
    geometry_msgs::PoseStamped position;
    position.pose.position.x = obstacles->points[k].x;
    position.pose.position.y = obstacles->points[k].y;
    position.header.stamp.now();
    position.header.frame_id = "batman/base_laser_link";
    pointPub.publish(position);
    */

    /* ważny test, który wykazał, że jak spotkamy punkt z dwoma zerami, to do końca tablicy wszystkie już takie będą, ale przy każdym skanie zera zaczynają się gdzie indziej
    bool zero=false;
    for(int i =0;i<obstacles->points.capacity();i++){
        if(obstacles->points[i].x==0.0 && obstacles->points[i].y==0.0)
        {
            if(!zero)
                std::cout<<i<<std::endl;
            zero = true;
        }else{
            if(zero){
                 std::cout<<obstacles->points[i];
            }
        }
    }*/
    return isFree;
}
void makeMarker(geometry_msgs::PoseStamped pos, float r, float g, float b){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "batman/base_laser_link";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pos.pose.position.x;
    marker.pose.position.y = pos.pose.position.y;
    marker.pose.position.z = pos.pose.position.z;
    marker.pose.orientation.x = pos.pose.orientation.x;
    marker.pose.orientation.y = pos.pose.orientation.y;
    marker.pose.orientation.z = pos.pose.orientation.z;
    marker.pose.orientation.w = pos.pose.orientation.w;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    markerViz.publish( marker );
}
void makeMarker(float x, float y, float r, float g, float b){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "batman/base_laser_link";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    markerViz.publish( marker );
}
