#ifndef MapMaker_h
#define MapMaker_h

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

template <typename T> struct Position{
    T x;
    T y;
    Position(T _x, T _y){
        x=_x;
        y=_y;
    }
};
struct myMap{
private:
	uint8_t* map = NULL;
public: 	
	int sizeX;
	int sizeY;
    float sizeBlock;
    int Length;
	myMap(){
		sizeX = 0;
        sizeY = 0;
		map = NULL;	
	}
    myMap(int a, float blockSize){
		map = new uint8_t[(a+1)*(2*a+1)];
   		sizeX = a+1;
        sizeY = 2*a+1;
        sizeBlock = blockSize;
        Length = sizeX*sizeY;
	}
	~myMap(){
		delete[] map;
	}
	uint8_t GetMap(int x, int y){
		if(x>=0&&y>=0&&x<sizeX&&y<sizeY){
			return map[x+y*sizeX];	
		}
		else{
			std::cout<<"Nie ma takiego miejsca na mapie!"<<std::endl;
            return 0;
        }
	}
    uint8_t GetMap(Position<int> pos){
        if(pos.x>=0&&pos.y>=0&&pos.x<sizeX&&pos.y<sizeY){
            return map[pos.x+pos.y*sizeX];
        }
        else{
            std::cout<<"Nie ma takiego miejsca na mapie!"<<std::endl;
            return 0;
        }
    }
    uint8_t GetMap(int i){
        if(i>=0&&i<Length)
            return map[i];
        else{
            std::cout<<"Nie ma takiego miejsca na mapie!"<<std::endl;
            return 0;
        }
    }
    Position<float> getPos(int i){
        if(i>=0&&i<Length){
            return Position<float>((i)%sizeX*sizeBlock, (i/sizeX-sizeX+1)*sizeBlock);
        }else{
            std::cout<<"Nie ma takiego miejsca na mapie!"<<std::endl;
            return Position<float>(-1,-1);
        }
    }

    uint8_t GetMap(float x, float y){
        int xi = int((x)/sizeBlock);
        int yi = int((y)/sizeBlock+sizeX-1);
        if(x>=0)
            return GetMap(xi, yi);
        else
            return 0;
    }
	void SetMap(int x, int y, uint8_t val){
		if(x>=0&&y>=0&&x<sizeX&&y<sizeY){
			map[x+y*sizeX] = val;	
		}
		else{
			std::cout<<"Nie ma takiego miejsca na mapie!"<<std::endl;
		}
	}
    void SetMap(Position<int> pos, uint8_t val){
        if(pos.x>=0&&pos.y>=0&&pos.x<sizeX&&pos.y<sizeY){
            map[pos.x+pos.y*sizeX] = val;
        }
        else{
            std::cout<<"Nie ma takiego miejsca na mapie!"<<std::endl;
        }
    }
    void SetMap(float x, float y, uint8_t val){
        int xi = int((x)/sizeBlock);
        int yi = int((y)/sizeBlock+sizeX-1);
        if(x>=0)
            SetMap(xi, yi, val);
    }
};
void FillOccupancyMap(myMap *&map, sensor_msgs::PointCloud* cloud, float bucketSize, float maxDist);
void FillOccupancyMap2(myMap *&map, sensor_msgs::PointCloud* cloud, float bucketSize, float maxDist);
void drawMap(myMap * map, ros::Publisher pub);
int numberNeig(myMap *map, Position<int> here, int step);
#endif
