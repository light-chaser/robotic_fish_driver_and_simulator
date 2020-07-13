/*
    file name           :   locomotion_change.cpp
    Copyright           :   empty
    Module name         :   locomotion_change

    create date         :   2020/3/5
    Author              :   zhang yujie

    Abstract            :   get the desire motion and drive CPG to get there

*/

#include<ros/ros.h>
#include<std_msgs/Float64MultiArray.h>

#include"../../../devel/include/cpg_driver_msg/cpg_param_change.h"
#include"../../../devel/include/cpg_driver_simulation/desire_direction.h"
#include <sensor_msgs/Imu.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_eigen/tf2_eigen.h>

#ifndef pi
#define pi 3.14
#endif


cpg_driver_simulation::desire_direction direct;
double                                  theta=0,phi=0,eps;
tf2::Transform                          now_state;
Eigen::Isometry3d                       desire_state;
Eigen::Isometry3d                       now_state_eigen;      //鱼体当前姿态
Eigen::Isometry3d                       delta_desire_state_eigen;//需要转换的姿态幅度矩阵

Eigen::AngleAxisd                       Lie_alg_of_now_state;//姿态的切空间
Eigen::AngleAxisd                       Lie_alg_of_desire_state;//期望姿态的切空间
Eigen::Vector3d                         angular;//用于Pd控制


void callback1(const cpg_driver_simulation::desire_direction &data)//计算期望姿态表达
{
    //将期望姿态转为旋转矩阵
    tf2::Matrix3x3 test;
    tf2::Quaternion quat;
    geometry_msgs::Transform temp;


    test.setEulerZYX(data.theta_d,data.phi_d,0.0);
    test.getRotation(quat);
    temp.rotation.w=quat.w();
    temp.rotation.x=quat.x();
    temp.rotation.y=quat.y();
    temp.rotation.z=quat.z();
    delta_desire_state_eigen = tf2::transformToEigen(temp);
}


void call_back2(const sensor_msgs::Imu  &state)   //计算实际姿态
{
    geometry_msgs::Transform temp;
    temp.rotation=state.orientation;
    temp.translation.x=temp.translation.y=temp.translation.z=0;

    now_state_eigen = tf2::transformToEigen(temp);
    Lie_alg_of_now_state = Lie_alg_of_now_state.fromRotationMatrix(now_state_eigen.rotation());
}


int main(int argc,char**argv)
{
    ros::init(argc,argv,"module_change");
    ros::NodeHandle nh;

    ros::Subscriber              sub1    =   nh.subscribe("avoidance",10,&callback1);
    ros::Subscriber              sub2    =   nh.subscribe("/naro/imu",10,&call_back2);
    
    ros::Publisher               pub     =   nh.advertise<cpg_driver_msg::cpg_param_change>("module_change1",10);

    cpg_driver_msg::cpg_param_change param;
    ros::Rate                    loops(10);
    double                       kp=3,kd=1.3;   //pd参数
    Eigen::Vector3d err,err2,u;//误差、控制律
    err = err2 =err2.setZero();
    desire_state.setIdentity();
    while(ros::ok())
    {
        ros::spinOnce();
        desire_state = desire_state * delta_desire_state_eigen;
        Lie_alg_of_desire_state = Lie_alg_of_desire_state.fromRotationMatrix(desire_state.rotation());

        err2 = err;
        err   =   Lie_alg_of_desire_state.axis()*Lie_alg_of_desire_state.angle()  \
                - Lie_alg_of_now_state.axis()*Lie_alg_of_now_state.angle();
        
        u     = -(err * kp +(err-err2)*kd) ;     //   pid控制
        
        std::cout<<"norm(err)="<< err <<std::endl<<std::endl;        //检查u;
        param.X_ca=abs(u.z()) > 0.4 ? (u.z()>0 ? 0.4:-0.4) : 2.1*u.z();//   构造闭环
        param.A_ca= abs(param.X_ca)>=0.02 ? abs(param.X_ca)/2 : 0.0;
        param.A_pec_l=param.A_pec_r=0;
        param.alfa_pec_l=param.alfa_pec_r=  (abs(u.y())<0.04 && abs(u.z())>0.002) ? 1.0:0.08+u.y();
        param.freq  =   0.8;
        pub.publish(param);
        loops.sleep();
    }
    return 0;
}
