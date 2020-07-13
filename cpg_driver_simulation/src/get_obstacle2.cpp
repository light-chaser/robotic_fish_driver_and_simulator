/*
    file name:      get_obstacle.cpp
  description:      该代码用于本人的避障控制仿真实验，通过
                    超声传感器识别前方是否存在障碍物，并通过
                    本程序中算法改变步态，实现避障功能
      version:      V1.0(and only 1.0 for me unless someone want to change)

        input:      五个超声波传感器的数据
       output:      障碍物中心坐标及其半径(以TF的形式，parent：world，child:obstacle)
*/
//#define dt 0.02

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include "obstacle_avoidance.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "../../../devel/include/cpg_driver_simulation/obstacle.h"
#include <pthread.h>
sphere_position p0,p[4], ps[3][4]; //坐标(均基于鱼体的随体坐标系)
sensor_msgs::Range R[5], min;

uint8_t res=0, res_124=0, res_125=0, res_134=0, res_135 = 0;

//float err = 1.0; //两个传感器所允许的最大测量差据，大于该值则只关注较小的数据。
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;//上锁

void sonar_callback(const sensor_msgs::Range &R0) //正前方向超声波
{
    R[0] = R0;
    if (R0.range > 1)
    {
        res |= 0;
    }
    else
    {
        min = R0;
        res |= SONAR1_ENCODE;
    }
}
void sonar1_callback(const sensor_msgs::Range &R1) //左侧方向超声波
{
    R[1] = R1;
    if (R1.range > 1)
    {
        res |= 0;
    }
    else
    {
        res |= SONAR2_ENCODE;
        if (min.range < R1.range)
            min = R1;
    }
}
void sonar2_callback(const sensor_msgs::Range &R2) //右侧方向超声波
{
    R[2] = R2;
    if (R2.range > 1)
    {
        res |= 0;
    }
    else
    {
        res |= SONAR3_ENCODE;
        if (min.range < R2.range)
            min = R2;
    }
}
void sonar3_callback(const sensor_msgs::Range &R3) //上侧方向超声波
{
    R[3] = R3;
    if (R3.range > 1)
    {
        res |= 0;
    }
    else
    {
        res |= SONAR4_ENCODE;
        if (min.range < R3.range)
            min = R3;
    }
}
void sonar4_callback(const sensor_msgs::Range &R4) //下侧方向超声波
{
    R[4] = R4;
    if (R4.range > 1)
    {
        res |= 0;
    }
    else
    {
        res |= SONAR5_ENCODE;
        if (min.range < R4.range)
            min = R4;
    }
}

void* area1_computing(void*)//
{
    while(1){
    if(pthread_mutex_lock(&mutex) != 0)
        {
          perror("pthread1_mutex_lock");
          exit(EXIT_FAILURE);
        }
    res_124 = res & (SONAR1_ENCODE | SONAR2_ENCODE | SONAR4_ENCODE);
        switch (res_124)
        {
        case 0:
            p[0].R = 0.0;
            p[0].x = p[0].y = p[0].z = 10;
            break;
        case (SONAR1_ENCODE):
            p[0].R = 0.5;
            p[0].x = R[0].range;
            p[0].y = 0;
            p[0].z = 0;
            break;
        case (SONAR2_ENCODE):
            p[0].R = 0.5;
            p[0].x = R[1].range * cos(0.4);
            p[0].y = R[1].range * sin(0.4);
            p[0].z = 0;
            break;
        case (SONAR4_ENCODE):
            p[0].R = 0.5;
            p[0].x = R[3].range * cos(0.4);
            p[0].y = 0;
            p[0].z = R[3].range * sin(0.4);
            break;
        case (SONAR2_ENCODE|SONAR1_ENCODE):
            ps[0][0].x = R[0].range;
            ps[0][0].y = ps[0][0].z = 0;
            ps[1][0].x = R[1].range * cos(0.4);
            ps[1][0].y = R[1].range * sin(0.4);
            ps[1][0].z = 0;
            p[0].x = (ps[0][0].x + ps[1][0].x) / 2;
            p[0].y = (ps[0][0].y + ps[1][0].y) / 2;
            p[0].z = 0;
            p[0].R = sqrt(pow(p[0].x - ps[1][0].x, 2) + pow(p[0].y - ps[1][0].y, 2) + pow(p[0].z - ps[1][0].z, 2));
            break;

        case (SONAR4_ENCODE|SONAR1_ENCODE):
            ps[0][0].x = R[0].range;
            ps[0][0].y = ps[0][0].z = 0;
            ps[1][0].x = R[3].range * cos(0.4);
            ps[1][0].y = 0;
            ps[1][0].z = R[3].range * sin(0.4);
            p[0].x = (ps[0][0].x + ps[1][0].x) / 2;
            p[0].y = (ps[0][0].y + ps[1][0].y) / 2;
            p[0].z = 0;
            p[0].R = sqrt(pow(p[0].x - ps[1][0].x, 2) + pow(p[0].y - ps[1][0].y, 2) + pow(p[0].z - ps[1][0].z, 2));
            break;

        case (SONAR4_ENCODE|SONAR1_ENCODE|SONAR2_ENCODE):
            ps[0][0].x = R[0].range;
            ps[0][0].y = ps[0][0].z = 0;
            ps[1][0].x = R[1].range * cos(0.4);
            ps[1][0].y = R[1].range * sin(0.4);
            ps[1][0].z = 0;
            ps[2][0].x = R[3].range * cos(0.4);
            ps[2][0].y = 0;
            ps[2][0].z = R[3].range * sin(0.4);
            p[0] = solveCenterPointOfCircle(ps[0][0], ps[1][0], ps[2][0]);
            break;
        }
    if(pthread_mutex_unlock(&mutex) != 0)
        {
          perror("pthread1_mutex_unlock");
          exit(EXIT_FAILURE);
        }
    
    }
}
void* area2_computing(void*)//
{
    while(1){
    if(pthread_mutex_lock(&mutex) != 0)
        {
          perror("pthread2_mutex_lock");
          exit(EXIT_FAILURE);
        }
    res_125 = res & (SONAR1_ENCODE | SONAR2_ENCODE | SONAR5_ENCODE);
        switch (res_125)
        {
        case 0:
            p[1].R = 0.0;
            p[1].x = p[1].y = p[1].z = 10;
            break;
        case (SONAR1_ENCODE):
            p[1].R = 0.5;
            p[1].x = R[0].range;
            p[1].y = 0;
            p[1].z = 0;
            break;
        case (SONAR2_ENCODE):
            p[1].R = 0.5;
            p[1].x = R[1].range * cos(0.4);
            p[1].y = R[1].range * sin(0.4);
            p[1].z = 0;
            break;
        case (SONAR5_ENCODE):
            p[1].R = 0.5;
            p[1].x = R[4].range * cos(0.4);
            p[1].y = 0;
            p[1].z = -R[4].range * sin(0.4);
            break;
        case (SONAR1_ENCODE|SONAR2_ENCODE):
            ps[0][1].x = R[0].range;
            ps[0][1].y = ps[0][1].z = 0;
            ps[1][1].x = R[1].range * cos(0.4);
            ps[1][1].y = R[1].range * sin(0.4);
            ps[1][1].z = 0;
            p[1].x = (ps[0][1].x + ps[1][1].x) / 2;
            p[1].y = (ps[0][1].y + ps[1][1].y) / 2;
            p[1].z = 0;
            p[1].R = sqrt(pow(p[1].x - ps[1][1].x, 2) + pow(p[1].y - ps[1][1].y, 2) + pow(p[1].z - ps[1][1].z, 2));
            break;
        case (SONAR1_ENCODE|SONAR5_ENCODE):
            ps[0][1].x = R[0].range;
            ps[0][1].y = ps[0][1].z = 0;
            ps[1][1].x = R[4].range * cos(0.4);
            ps[1][1].y = 0;
            ps[1][1].z = -R[4].range * sin(0.4);
            p[1].x = (ps[0][1].x + ps[1][1].x) / 2;
            p[1].y = (ps[0][1].y + ps[1][1].y) / 2;
            p[1].z = 0;
            p[1].R = sqrt(pow(p[1].x - ps[1][1].x, 2) + pow(p[1].y - ps[1][1].y, 2) + pow(p[1].z - ps[1][1].z, 2));

            break;
        case (SONAR1_ENCODE|SONAR2_ENCODE|SONAR5_ENCODE):
            ps[0][1].x = R[0].range;
            ps[0][1].y = ps[0][1].z = 0;
            ps[1][1].x = R[1].range * cos(0.4);
            ps[1][1].y = R[1].range * sin(0.4);
            ps[1][1].z = 0;
            ps[2][1].x = R[3].range * cos(0.4);
            ps[2][1].y = 0;
            ps[2][1].z = -R[3].range * sin(0.4);
            p[1] = solveCenterPointOfCircle(ps[0][1], ps[1][1], ps[2][1]);
            break;
        default:

            break;
        }
    if(pthread_mutex_unlock(&mutex) != 0)
        {
          perror("pthread_mutex_lock");
          exit(EXIT_FAILURE);
        }
        
    }
}
void* area3_computing(void*)//
{
    while(1){
            if(pthread_mutex_lock(&mutex) != 0)
        {
          perror("pthread_mutex_lock");
          exit(EXIT_FAILURE);
        }
    res_134 = res & (SONAR1_ENCODE | SONAR3_ENCODE | SONAR4_ENCODE);
    switch (res_134)
        {
        case 0:
            p[2].R = 0.0;
            p[2].x = p[2].y = p[2].z = 10;
            break;
        case (SONAR1_ENCODE):
            p[2].R = 0.5;
            p[2].x = R[0].range;
            p[2].y = 0;
            p[2].z = 0;
            break;
        case (SONAR3_ENCODE):
            p[2].R = 0.5;
            p[2].x = R[2].range * cos(0.4);
            p[2].y = -R[2].range * sin(0.4);
            p[2].z = 0;
            break;
        case (SONAR4_ENCODE):
            p[2].R = 0.5;
            p[2].x = R[3].range * cos(0.4);
            p[2].y = 0;
            p[2].z = R[3].range * sin(0.4);
            break;
        case (SONAR1_ENCODE|SONAR3_ENCODE):
            ps[0][2].x = R[0].range;
            ps[0][2].y = ps[0][2].z = 0;
            ps[1][2].x = R[2].range * cos(0.4);
            ps[1][2].y = -R[2].range * sin(0.4);
            ps[1][2].z = 0;
            p[2].x = (ps[0][2].x + ps[1][2].x) / 2;
            p[2].y = (ps[0][2].y + ps[1][2].y) / 2;
            p[2].z = 0;
            p[2].R = sqrt(pow(p[2].x - ps[1][2].x, 2) + pow(p[2].y - ps[1][2].y, 2) + pow(p[2].z - ps[1][2].z, 2));
            break;

        case (SONAR4_ENCODE|SONAR1_ENCODE):
            ps[0][2].x = R[0].range;
            ps[0][2].y = ps[0][2].z = 0;
            ps[1][2].x = R[3].range * cos(0.4);
            ps[1][2].y = 0;
            ps[1][2].z = R[3].range * sin(0.4);
            p[2].x = (ps[0][2].x + ps[1][2].x) / 2;
            p[2].y = (ps[0][2].y + ps[1][2].y) / 2;
            p[2].z = 0;
            p[2].R = sqrt(pow(p[2].x - ps[1][2].x, 2) + pow(p[2].y - ps[1][2].y, 2) + pow(p[2].z - ps[1][2].z, 2));

            break;

        case (SONAR1_ENCODE|SONAR3_ENCODE|SONAR4_ENCODE):
            ps[0][2].x = R[0].range;
            ps[0][2].y = ps[0][2].z = 0;
            ps[1][2].x = R[1].range * cos(0.4);
            ps[1][2].y = -R[1].range * sin(0.4);
            ps[1][2].z = 0;
            ps[2][2].x = R[3].range * cos(0.4);
            ps[2][2].y = 0;
            ps[2][2].z = R[3].range * sin(0.4);
            p[2] = solveCenterPointOfCircle(ps[0][2], ps[1][2], ps[2][2]);
            break;
        default:
            break;
        }
    if(pthread_mutex_unlock(&mutex) != 0)
        {
          perror("pthread_mutex_lock");
          exit(EXIT_FAILURE);
        }
        
    }

}
void* area4_computing(void*)//
{
    while(1){
    if(pthread_mutex_lock(&mutex) != 0)
        {
          perror("pthread_mutex_lock");
          exit(EXIT_FAILURE);
        }
    res_135 = res & (SONAR1_ENCODE | SONAR3_ENCODE | SONAR5_ENCODE);
    switch (res_135)
        {
        case 0:
            p[3].R = 0.0;
            p[3].x = p[3].y = p[3].z = 10;
            break;
        case (SONAR1_ENCODE):
            p[3].R = 0.5;
            p[3].x = R[0].range;
            p[3].y = 0;
            p[3].z = 0;
            break;
        case (SONAR3_ENCODE):
            p[3].R = 0.5;
            p[3].x = R[2].range * cos(0.4);
            p[3].y = -R[2].range * sin(0.4);
            p[3].z = 0;
            break;
        case (SONAR5_ENCODE):
            p[3].R = 0.5;
            p[3].x = R[4].range * cos(0.4);
            p[3].y = 0;
            p[3].z = -R[4].range * sin(0.4);
            break;

        case (SONAR3_ENCODE|SONAR1_ENCODE):
            ps[0][3].x = R[0].range;
            ps[0][3].y = ps[0][3].z = 0;
            ps[1][3].x = R[2].range * cos(0.4);
            ps[1][3].y = -R[2].range * sin(0.4);
            ps[1][3].z = 0;
            p[3].x = (ps[0][3].x + ps[1][3].x) / 2;
            p[3].y = (ps[0][3].y + ps[1][3].y) / 2;
            p[3].z = 0;
            p[3].R = sqrt(pow(p[3].x - ps[1][3].x, 2) + pow(p[3].y - ps[1][3].y, 2) + pow(p[3].z - ps[1][3].z, 2));

            break;

        case (SONAR1_ENCODE|SONAR5_ENCODE):
            ps[0][3].x = R[0].range;
            ps[0][3].y = ps[0][3].z = 0;
            ps[1][3].x = R[4].range * cos(0.4);
            ps[1][3].y = 0;
            ps[1][3].z = -R[4].range * sin(0.4);
            p[3].x = (ps[0][3].x + ps[1][3].x) / 2;
            p[3].y = (ps[0][3].y + ps[1][3].y) / 2;
            p[3].z = 0;
            p[3].R = sqrt(pow(p[3].x - ps[1][3].x, 2) + pow(p[3].y - ps[1][3].y, 2) + pow(p[3].z - ps[1][3].z, 2));
            break;

        case (SONAR5_ENCODE|SONAR1_ENCODE|SONAR3_ENCODE):
            ps[0][3].x = R[0].range;
            ps[0][3].y = ps[0][0].z = 0;
            ps[1][3].x = R[1].range * cos(0.4);
            ps[1][3].y = R[1].range * sin(0.4);
            ps[1][3].z = 0;
            ps[2][3].x = R[3].range * cos(0.4);
            ps[2][3].y = 0;
            ps[2][3].z = -R[3].range * sin(0.4);
            p[3] = solveCenterPointOfCircle(ps[0][3], ps[1][3], ps[2][3]);
            break;
        default:

            break;
        }
    if(pthread_mutex_unlock(&mutex) != 0)
        {
          perror("pthread_mutex_lock");
          exit(EXIT_FAILURE);
        }       
    }
}





    
    
int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_avoidance");
    ros::NodeHandle nh;

    ros::Subscriber sonar_sub = nh.subscribe("/naro/sonar", 10, &sonar_callback);
    ros::Subscriber sonar1_sub = nh.subscribe("/naro/sonar1", 10, &sonar1_callback);
    ros::Subscriber sonar2_sub = nh.subscribe("/naro/sonar2", 10, &sonar2_callback);
    ros::Subscriber sonar3_sub = nh.subscribe("/naro/sonar3", 10, &sonar3_callback);
    ros::Subscriber sonar4_sub = nh.subscribe("/naro/sonar4", 10, &sonar4_callback);

    ros::Publisher obstacle_pub = nh.advertise<cpg_driver_simulation::obstacle>("obstacles", 100);
    ros::Rate loop(50);
    cpg_driver_simulation::obstacle ost;
    pthread_t id1, id2, id3, id4;

    if (pthread_create(&id1, NULL, area1_computing, NULL) != 0)
    {
        ROS_INFO("Threads 1 creation failed.");
        return (1);
    }
    if (pthread_create(&id2, NULL, area2_computing, NULL) != 0)
    {
        ROS_INFO("Threads 2 creation failed.");
        return (1);
    }
    if (pthread_create(&id3, NULL, area3_computing, NULL) != 0)
    {
        ROS_INFO("Threads 3 creation failed.");
        return (1);
    }
    if (pthread_create(&id4, NULL, area4_computing, NULL) != 0)
    {
        ROS_INFO("Threads 4 creation failed.");
        return (1);
    }
    sphere_position p_t1,p_t2;
    while (ros::ok())
    {
//        std::cout << (int)res << std::endl;
        if(pthread_mutex_lock(&mutex) != 0)
        {
          perror("pthread_mutex_lock");
          exit(EXIT_FAILURE);
        }
        p_t1=(  
                sqrt(pow(p[0].x,2)+pow(p[0].y,2)+pow(p[0].z,2))\
                    <       \
                sqrt(pow(p[1].x,2)+pow(p[1].y,2)+pow(p[1].z,2))\
                )? p[0]:p[1];
        p_t2=(  
                sqrt(pow(p[2].x,2)+pow(p[2].y,2)+pow(p[2].z,2))\
                    <       \
                sqrt(pow(p[3].x,2)+pow(p[3].y,2)+pow(p[3].z,2))\
                )? p[2]:p[3];
        p0=(  
                sqrt(pow(p_t1.x,2)+pow(p_t1.y,2)+pow(p_t1.z,2))\
                    <       \
                sqrt(pow(p_t2.x,2)+pow(p_t2.y,2)+pow(p_t2.z,2))\
                )? p_t1:p_t2;

        ost.R = p0.R<0.5 ? 0.5 : p0.R;
        ost.x = p0.x;
        ost.y = p0.y;
        ost.z = p0.z;
        obstacle_pub.publish(ost);
        ros::spinOnce();
        if(pthread_mutex_unlock(&mutex) != 0)
        {
          perror("pthread_mutex_lock");
          exit(EXIT_FAILURE);
        }
        loop.sleep();
    }
}
