//MATH LIB
#include<iostream>
#include<eigen3/Eigen/Dense>

//ROS LIB & THE MSG FILE
#include<ros/ros.h>
#include<sensor_msgs/Imu.h>
#include"../../../devel/include/cpg_driver_msg/cpg_param_change.h"
#include<geometry_msgs/Quaternion.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>
#include<tf2/transform_datatypes.h>
//MULTI-THREADS
#include<pthread.h>

//变量//
//期望、实际输出、输出微分、误差、误差导数、控制律
double yd,y,d_y,err,d_err,u;
//RBF相关变量
using namespace Eigen;
using namespace std;
//rbf神经网络输入、输出
Vector3d RBF_input,RBF_OUTPUT;
//隐含层矩阵
Matrix<double,3,17> RBF_hidden_param;
Matrix<double,17,1> RBF_hidden;
const double b=0.2;
const double c0=0.02;
//输出层权重
Matrix<double,17,3> RBF_weight,RBF_update;
//互斥锁
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;


//接收姿态信息
void pitching(const sensor_msgs::Imu &imu)
{
  double roll,pitch,yaw;
  tf2::Quaternion quat(imu.orientation.x,\
                       imu.orientation.y,\
                       imu.orientation.z,\
                       imu.orientation.w);
  tf2::Matrix3x3(quat).getRPY(roll,pitch,yaw);
  y   = pitch;
  d_y = imu.angular_velocity.y;
}
//RBF权重自适应计算
void *adapt_RBF(void *);
//RBF输出计算
void *RBF(void *);      

int main(int argc,char **argv)
{
    ros::init(argc,argv,"RBF_pitching");
    ros::NodeHandle nh;
    ros::Subscriber sub1    =   nh.subscribe("naro/imu",10,&pitching);
//    ros::Publisher  pub     =   nh.advertise<cpg_driver_msg::cpg_param_change>("module_change1",10);//simulation
    ros::Publisher  pub     =   nh.advertise<cpg_driver_msg::cpg_param_change>("module_change1",10);//real robots

    pthread_t t1,t2;
    if (pthread_create(&t1, NULL, adapt_RBF, NULL) != 0)
    {
        ROS_INFO("Threads 1 creation failed.");
        return (1);
    }
    if (pthread_create(&t2, NULL, RBF, NULL) != 0)
    {
        ROS_INFO("Threads 2 creation failed.");
        return (1);
    }
    yd=0.7;
    //PD参数
    double kp=1,kd=0.6;
    //CPG参_msg数
    cpg_driver_msg::cpg_param_change param;
    param.freq = 1.3;
    param.A_pec_l=param.A_pec_r=0;
    param.alfa_pec_l=param.alfa_pec_r=0;
    param.A_ca=0.2;
    param.X_ca=0;

    ros::Rate loop(param.freq);
    //初始值//
    RBF_input=RBF_input.Zero();   RBF_weight=RBF_weight.Zero();
    RBF_OUTPUT=RBF_OUTPUT.Zero(); RBF_update=RBF_update.Zero();
    RBF_hidden_param <<
                        1.57,1.4,1.3,1.2,1.1,1.0,0.8,0.5,0,-1.57,-1.4,-1.3,-1.2,-1.1,-1.0,-0.8,-0.5, 
                        2.0 ,1.8,1.6,1.4,1.2,1.0,0.8,0.3,0,-2.0 ,-1.8,-1.6,-1.4,-1.2,-1.0,-0.8,-0.3, 
                        1.57,1.4,1.3,1.2,1.1,1.0,0.8,0.5,0,-1.57,-1.4,-1.3,-1.2,-1.1,-1.0,-0.8,-0.5; 
    u = 0;
    while(ros::ok())
    {
        //上锁
        if(pthread_mutex_lock(&mutex) != 0)
        {
          perror("pthread_mutex_lock");
          exit(EXIT_FAILURE);
        }
        //计算误差
        err     = yd - y;   
        d_err   = -d_y;
        //控制律计算
        u       =- (RBF_OUTPUT[0]*y+RBF_OUTPUT[1]*d_y+kp*err+kd*d_err)/RBF_OUTPUT[2];
        u       = abs(u)<1.0? u : (u>0 ? 1.0:-1.0);
        param.alfa_pec_l=param.alfa_pec_r=u;
        //发布
        pub.publish(param);
        //解锁
        ros::spinOnce();
        loop.sleep();
        if(pthread_mutex_unlock(&mutex) != 0)
        {
          perror("pthread_mutex_lock");
          exit(EXIT_FAILURE);
        }
        

    }
}


void *RBF(void *)
{
  while(1){
  if(pthread_mutex_lock(&mutex) != 0)
  {
      perror("pthread_mutex_lock");
      exit(EXIT_FAILURE);
  }

  RBF_input<<y,d_y,u;//输入

  for(int i=0;i<RBF_hidden.size();i++)//隐含侧计算
  {
    Vector3d temp=RBF_input-RBF_hidden_param.col(i);
    RBF_hidden[i]=exp(temp.norm()/2*pow(b,2));
  }
  RBF_OUTPUT=RBF_weight.transpose()*RBF_hidden;//输出
  RBF_OUTPUT[2] = abs(RBF_OUTPUT[2])>=0.02 ? RBF_OUTPUT[2]: \
                  (RBF_OUTPUT[2]>0 ? c0 : -c0 );


  if(pthread_mutex_unlock(&mutex) != 0)
  {
      perror("pthread_mutex_lock");
      exit(EXIT_FAILURE);
  }
  }
}
void *adapt_RBF(void *)
{
  while(1){
    if(pthread_mutex_lock(&mutex) != 0)
    {
      perror("pthread_mutex_lock");
      exit(EXIT_FAILURE);
    }

    const double gamma=50;
    const Matrix2d Q=Q.Identity();
    const Matrix2d P;//怎么算？
    Vector2d B;
    B << 0,1;
    Vector2d E;E << err,d_err;

    RBF_update = E.transpose()*P*B*RBF_hidden*RBF_input.transpose()*gamma;
    RBF_weight += RBF_update/1.45;
    if(pthread_mutex_unlock(&mutex) != 0)
    {
        perror("pthread_mutex_lock");
        exit(EXIT_FAILURE);
    }
  }
}
