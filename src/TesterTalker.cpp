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

#include "MapMaker.h"
#include "MakeMarker.h"

///global variables
geometry_msgs::Pose goalPosition;
geometry_msgs::Pose currentPosition;
const double tf::Transformer::DEFAULT_CACHE_TIME = 10.0;
laser_geometry::LaserProjection projector;
//ros::Publisher pointPub;
ros::Publisher markerViz;
float robotWidth = 0.5;//ważny parametr grubości naszego ulubieńca najlepiej podawać wraz z zapasem odległości od przeszkód
bool Ruszaj_Batmobilu = false;
tf::TransformListener* listener;

uint8_t occupancyGrid[51][101];
myMap* occupancyMap = NULL;

///function dectarations
void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scanMsg);
void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
void UpdateCurrentPosition(geometry_msgs::Pose &newPose);
std::vector <geometry_msgs::Vector3> Find_Points(sensor_msgs::PointCloud* cloud, const sensor_msgs::LaserScan::Ptr& scan);
bool isPathFree(geometry_msgs::Pose there, sensor_msgs::PointCloud* obstacles);
void FillOccupancyGrid(uint8_t grid[][101], sensor_msgs::PointCloud* cloud, int bucketsNumber, float bucketsSize);

//void FillOccupancyMap(uint8 *map, sensor_msgs::PointCloud* cloud, float bucketSize, float maxDist);
///main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "LocalPlanner");

    ros::NodeHandle n;
    ros::Subscriber subScan = n.subscribe("/batman/scan", 100, ScanCallback);
    ros::Subscriber goalSub = n.subscribe("/move_base_simple/goal", 20, GoalCallback);
    //pointPub = n.advertise<geometry_msgs::PoseStamped>("/path/chwilowy", 1000);
    markerViz = n.advertise<visualization_msgs::Marker>("/markers", 1000);
    listener = new tf::TransformListener();
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        //UpdateCurrentPosition(currentPosition);

        ros::spinOnce();

        loop_rate.sleep();
    }
    delete listener;
    return 0;
}


void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scanMsg)
{
    int scanNumber = scanMsg->scan_time / scanMsg->time_increment;
    float maxValue = 5.0;
    sensor_msgs::LaserScan::Ptr newMsg (new sensor_msgs::LaserScan(*scanMsg));

    for(int i=0;i<scanMsg->ranges.size();i++){
        if(std::isnan(scanMsg->ranges[i])) {
            newMsg->ranges[i] = maxValue;
        }
    }
    sensor_msgs::PointCloud cloud;
    sensor_msgs::LaserScan::ConstPtr ptr (newMsg);

    projector.projectLaser(*ptr, cloud);
    //to nie jest miejsce na wywołanie tej funkcji, ale do testu się nadaje
    isPathFree(goalPosition, &cloud);

    // // FillOccupancyGrid(occupancyGrid, &cloud, 51, 0.1);

    FillOccupancyMap2(occupancyMap, &cloud, 0.1, maxValue);
    drawMap(occupancyMap, markerViz);
    //Find_Points(&cloud, newMsg);
}

void TfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{

}

void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    //przeliczam goal na współrzędne robota
    tf::Stamped<tf::Pose> goal, transformed_goal;
    geometry_msgs::PoseStamped transformed_goalMsg;
    tf::poseStampedMsgToTF(*goalMsg, goal);
    try
    {
        listener->waitForTransform("batman/base_laser_link", goal.frame_id_, ros::Time::now(), ros::Duration(3.0));
        listener->transformPose("batman/base_laser_link", goal, transformed_goal);

        tf::poseStampedTFToMsg(transformed_goal, transformed_goalMsg);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    ///copy the given goal to global variable
    memcpy(&goalPosition, &(transformed_goalMsg.pose), sizeof(goalPosition));

    float x = transformed_goalMsg.pose.position.x;
    float y = transformed_goalMsg.pose.position.y;
    float z = transformed_goalMsg.pose.position.z;
    printf("goal: x, y, z: %f, %f, %f\n", x, y, z);

}

void UpdateCurrentPosition(geometry_msgs::Pose &newPose)
{
    tf::StampedTransform transform;

    try
    {
        listener->lookupTransform("batman/odom", "batman/base_laser_link", ros::Time(0), transform);
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
//miało znaleźć punkty pośrednie do jazdy
std::vector <geometry_msgs::Vector3> Find_Points(sensor_msgs::PointCloud* cloud, const sensor_msgs::LaserScan::Ptr&  scan){
    std::vector <geometry_msgs::Vector3> points;
    int center = 0, points_num = 0;
    float centerAngle = 0.0;


    centerAngle = (scan->angle_min) ;
    std::cout<<"min "<< scan->angle_min <<"  incr " << scan->angle_increment << "center " << centerAngle
            << " max "<< scan->angle_max<<std::endl;


    center = int(std::abs( centerAngle / scan->angle_increment));

    std::cout<<center<<" ("<<cloud->points[center].x<<", "<<cloud->points[center].y<<")"<<std::endl;
    std::cout<<scan->ranges[center]<<std::endl<<std::endl;
    /*for(int i = 0; i<cloud->points.size(); i++){
        //kończy pętle, gdy dojdzie do pustych punktów
        if(cloud->points[i].x==0.0 && obstacles->points[i].y==0.0){
            break;
        }
        if(i>0){
            if(obstacles->points[i-1].y*obstacles->points[i].y<0){
                center=i;
                l++;
            }
        }
    }*/
    return points;
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
    for(int i =0;i<obstacles->points.size();i++){
        //kończy pętle, gdy dojdzie do pustych punktów
        if(obstacles->points[i].x==0.0 && obstacles->points[i].y==0.0){
            break;
        }
        //sprawdza, czy punkt nie znajduje się za robotem
        if(obstacles->points[i].x<0){
           //nic nie robi, jak jest za robotem - zakres skanera zdaje się być większy niż 180 stopni
        }else{
            if(obstacles->points[i].x<distance && std::abs(obstacles->points[i].y)<robotWidth/2){
                isFree = false;
               // std::cout<<"punkt na drodze nr "<<i<<std::endl;
                makeMarker(markerViz, i, obstacles->points[i].x, obstacles->points[i].y, 1,1,1);
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
    for(int i =0;i<obstacles->points.size();i++){
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

// 0 - puste
// 1 - przeszkoda
// 2 - homoniewiadomo
void FillOccupancyGrid(uint8_t grid[][101], sensor_msgs::PointCloud* cloud, int bucketsNumber, float bucketsSize) {
    ///idzie po pierścieniu
    int yC = bucketsNumber / 2;
    int xC = bucketsNumber / 2;

    ///fill the grid with '2'
    for (int i = 0; i < bucketsNumber; ++i) {
        for (int j = 0; j < bucketsNumber; ++j) {
            grid[i][j] = 2;
        }
    }

    int tr = bucketsNumber / 2;

    ///put the points in point cloud in grid
    for (int i = 0; i < cloud->points.size(); ++i) {
        if (cloud->points[i].x <= 3.0 && cloud->points[i].x >= 0 && std::abs(cloud->points[i].y <= 3.0)) {
            int x = tr - (cloud->points[i].x / bucketsSize);
            int y = tr - (cloud->points[i].y / bucketsSize);
            //printf("trans: %d, x: %d, y: %d \n", translation, x, y);
            grid[x][y] = 1;
            makeMarker(markerViz, i + 1000, cloud->points[i].x, cloud->points[i].y, 1, 0, 0);
        }
    }

    ///now fill the zeroes in map
    for (int i = 1; i < bucketsNumber / 2 - 1; ++i) {
        if(grid[xC + i + 1][yC] == 1) {
            break;
        } else {
            grid[xC + i][yC] = 0;
            makeMarker(markerViz, i + 2000, cloud->points[i].x, cloud->points[i].y, 0, 0, 1);
        }

        if(grid[xC][yC + i + 1] == 1) {
            break;
        } else {
            grid[xC][yC + 1] = 0;
            makeMarker(markerViz, i + 3000, cloud->points[i].x, cloud->points[i].y, 0, 0, 1);
        }

        for (int j = 1; j < i; ++j) {
            if(grid[xC + j][yC + i] == 1)
                continue;
            if(grid[xC + j][yC + i + 1] == 1 || grid[xC + j + 1][yC + i + 1] == 1 || grid[xC + j - 1][yC + i + 1] == 1) {
                grid[xC + j][yC + i] = 2;
            } else {
                grid[xC + j][yC + i] = 0;
                makeMarker(markerViz, i + 2000, cloud->points[i].x, cloud->points[i].y, 0, 0, 1);
            }

        }

        for (int j = 1; j < i; ++j) {

        }
    }

    ///print the result
    for (int i = 0; i < bucketsNumber; ++i) {
        for (int j = 0; j < bucketsNumber; ++j) {
            printf("%d ", grid[i][j]);
        }
        printf("\n");
    }
    printf("\n\n\n");
}
