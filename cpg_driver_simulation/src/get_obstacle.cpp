/*
    file name:      get_obstacle.cpp
  description:      该代码用于本人的避障控制仿真实验，通过
                    超声传感器识别前方是否存在障碍物，并通过
                    本程序中算法改变步态，实现避障功能
      version:      V1.0(and only 1.0 for me unless someone want to change)

        input:      三个超声波传感器的数据
       output:      障碍物中心坐标及其半径(以TF的形式，parent：world，child:obstacle)
*/
//#define dt 0.02


#include<ros/ros.h>
#include <sensor_msgs/Range.h>
#include"obstacle_avoidance.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "../../../devel/include/cpg_driver_simulation/obstacle.h"
sphere_position p1,p2,p3,p4,p5,p;//坐标(均基于鱼体的随体坐标系)

void sonar_callback(const sensor_msgs::Range &R)//正前方向超声波
{
    if(R.range<3){
        p1.x=R.range;
        p1.y=0;
        p1.z=0;
    }
    else{
        p1.x=0;//该方向上无障碍物
        p1.y=0;
        p1.z=0;
    }
}
void sonar1_callback(const sensor_msgs::Range &R1)//左侧方向超声波
{
    if(R1.range<3){
        p2.x=R1.range*cos(0.4);
        p2.y=R1.range*sin(0.4);
        p2.z=0;
        std::cout<<"p2.x="<<p2.x<<",p2.y="<<p2.y<<std::endl;
    }
    else{
        p2.x=0;//该方向上无障碍物
        p2.y=0;
        p2.z=0;
    }
}
void sonar2_callback(const sensor_msgs::Range &R2)//右侧方向超声波
{
    if(R2.range<3){
        p3.x=R2.range*cos(0.4);
        p3.y=-R2.range*sin(0.4);
        std::cout<<"p3.x="<<p3.x<<",p3.y="<<p3.y<<std::endl;
        p3.z=0;
    }
    else{
        p3.x=0;//该方向上无障碍物
        p3.y=0;
        p3.z=0;
    }
}
void sonar3_callback(const sensor_msgs::Range &R3)//右侧方向超声波
{
    if(R3.range<3){
        p4.x=R3.range*cos(0.4);
        p4.y=-R3.range*sin(0.4);
        std::cout<<"p4.x="<<p4.x<<",p4.y="<<p4.y<<std::endl;
        p4.z=0;
    }
    else{
        p4.x=0;//该方向上无障碍物
        p4.y=0;
        p4.z=0;
    }
}
void sonar4_callback(const sensor_msgs::Range &R4)//右侧方向超声波
{
    if(R4.range<3){
        p5.x=R4.range*cos(0.4);
        p5.y=-R4.range*sin(0.4);
        std::cout<<"p5.x="<<p5.x<<",p5.y="<<p5.y<<std::endl;
        p5.z=0;
    }
    else{
        p5.x=0;//该方向上无障碍物
        p5.y=0;
        p5.z=0;
    }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"obstacle_avoidance");
    ros::NodeHandle nh;

    ros::Subscriber sonar_sub   =   nh.subscribe("/naro/sonar",10,&sonar_callback);
    ros::Subscriber sonar1_sub  =   nh.subscribe("/naro/sonar1",10,&sonar1_callback);
    ros::Subscriber sonar2_sub  =   nh.subscribe("/naro/sonar2",10,&sonar2_callback);

    ros::Publisher  obstacle_pub=   nh.advertise<cpg_driver_simulation::obstacle>("obstacles",100);

    ros::Rate loop(50);

    cpg_driver_simulation::obstacle ost;
    
    int mask=0;
    while(ros::ok()){
        ros::spinOnce();

        mask=((abs(p2.x)>0.01)<<2) | ((abs(p1.x)>0.01)<<1) | (abs(p3.x)>0.01);//?????//
        std::cout<<"mask="<<mask<<std::endl;
        switch (mask){
            case(0b000):{
            p.R=0;
            break;
        }
            case(0b001):{
            p=p3;p.R=0.3;
            break;
        }
            case(0b010):{
            p=p1;p.R=0.3;
            break;
        }
            case(0b100):{
            p=p2;p.R=0.3;
            break;
        }
            case(0b011):{
                p.x=(p1.x+p3.x)/2;
                p.y=(p1.y+p3.y)/2;
                p.z=(p1.z+p3.z)/2;
                p.R=sqrt(pow(p.x-p1.x,2)+pow(p.y-p1.y,2)+pow(p.z-p1.z,2));
                if(p.R>0.3) p.R=0.3;
                break;            
        }
            case(0b110):{
                p.x=(p1.x+p2.x)/2;
                p.y=(p1.y+p2.y)/2;
                p.z=(p1.z+p2.z)/2;
                p.R=sqrt(pow(p.x-p1.x,2)+pow(p.y-p1.y,2)+pow(p.z-p1.z,2));
                if(p.R>0.3) p.R=0.3;
                break;
        }
            case(0b111):
                p=solveCenterPointOfCircle(p1,p2,p3);//三个点全部测有障碍物
                break;
            default:
                p.x=p.y=p.z=p.R=0;
                break;
        }
            ost.R   =  p.x;
            ost.x   =  p.x;
            ost.y   =  p.y;
            ost.z   =  p.z;
            obstacle_pub.publish(ost);
        mask=0;
        loop.sleep();
    }
}



