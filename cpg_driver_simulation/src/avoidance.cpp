/*
    file name:      avoidance.cpp
  description:      该代码用于本人的避障控制仿真实验，通过
                    超声传感器识别前方是否存在障碍物，并通过
                    本程序中算法改变步态，实现避障功能
      version:      V1.0(and only 1.0 for me unless someone want to change)

        input:      障碍物的位置坐标及其半径
       output:      CPG控制参数
*/

#include<ros/ros.h>
#include<std_msgs/Float64MultiArray.h>
#include "../../../devel/include/cpg_driver_simulation/obstacle.h"
#include<gazebo_msgs/ModelStates.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/transform_datatypes.h>
#include"../../../devel/include/cpg_driver_simulation/desire_direction.h"
#ifndef pi
#define pi 3.14
#endif




tf2::Vector3 V,V_b;//鱼体速度
float theta,phi,r_square;//视线坐标系偏转、俯仰角，相对距离
float R;        //障碍物半径



void call_back1(const cpg_driver_simulation::obstacle &ost)//get the direction of obstacles
{
    R            = ost.R;
    if(R<0.1) 
              R  = 0.15;
    theta        = atan2(ost.y,ost.x-0.15);
    phi          = atan2(ost.z,sqrt(pow(ost.x-0.15,2)+pow(ost.y,2)));
    if(theta>=pi/2)         {theta-=pi;}
    else if(theta<=-pi/2)   {theta+=pi;}
    if(phi>=pi/2)           {phi-=pi;}
    else if(phi<=-pi/2)     {phi+=pi;}


//    std::cout<<"theta="<<theta<<','<<"phi="<<phi<<std::endl;
    r_square = pow(ost.x-0.25,2)+pow(ost.y,2)+pow(ost.z,2);
}
void call_back2(const gazebo_msgs::ModelStates  &state)   //get the velocity of fish
{
    V.setX(state.twist[1].linear.x);
    V.setY(state.twist[1].linear.y);
    V.setZ(state.twist[1].linear.z);
}






int main(int argc,char**argv){
  ros::init(argc,argv,"avoidance");
  ros::NodeHandle nh;
  ros::Subscriber sub1                 = nh.subscribe("obstacles",10,&call_back1);//obstacles
  ros::Subscriber sub2                 = nh.subscribe("gazebo/model_states",10,&call_back2);//velocity
  ros::Publisher  pub                  = nh.advertise<cpg_driver_simulation::desire_direction>("avoidance",10);
  tf2::Matrix3x3 trans,trans2;             //转换矩阵

//订阅坐标变换//
  tf2_ros::Buffer buffer;             
  tf2_ros::TransformListener tfl(buffer);
  ros::Time time =ros::Time::now();
  ros::Duration timeout(10.0);

  geometry_msgs::TransformStamped tfGeom;
  try {
      tfGeom = buffer.lookupTransform("camera", "world", time, timeout);
  } 
  catch (tf2::TransformException &e) {
     // # handle lookup error
  }

  tf2::Quaternion quat1,quat2;
//订阅坐标变换//

  double theta_v,phi_v;//速度方向角
  cpg_driver_simulation::desire_direction data;//期望方向
  ros::Rate loop(50); 
  int n=0;
  //避障算法//
  while(ros::ok())
  {
    ros::spinOnce();
    tf2::fromMsg(tfGeom.transform.rotation, quat1);
    quat2.setEulerZYX(theta,phi,0.0);
    trans.setRotation(quat1);//旋转矩阵1
    trans2.setRotation(quat2);//旋转矩阵2

    V_b=trans2.transpose()*trans*V*-1;       //LOS速度
    std::cout<<V.getX()<<','<<V.getY()<<','<<V.getZ()<<std::endl<<std::endl;
    std::cout<<V_b.getX()<<','<<V_b.getY()<<','<<V_b.getZ()<<std::endl;
    //与LOS的偏角
    theta_v   = atan2(V_b.getY(),V_b.getX());
    phi_v     = atan2(V_b.getZ(),tf2Sqrt(tf2Pow(V_b.getY(),2)+tf2Pow(V_b.getX(),2)));
    //限定角度范围于[-pi/2,pi/2]//
    if(theta_v>=pi/2)         {theta_v-=pi;}
    else if(theta_v<=-pi/2)   {theta_v+=pi;}
    if(phi_v>=pi/2)           {phi_v-=pi;}
    else if(phi_v<=-pi/2)     {phi_v+=pi;}
    std::cout<<"theta_v="<<theta_v<<",phi_v="<<phi_v<<std::endl;
    double theta_m[2],phi_m[2],theta_min,phi_min;
    

    if(r_square * (tf2Pow(V_b.getY(),2) + tf2Pow(V_b.getZ(),2)) <  pow(R,2)*(V_b.length2()))/*壁碰条件:r^2*( V_theta^2 + V_phi^2 ) <= V^2 * R^2*/
    {
      theta_m[0]  = -asin(sqrt((pow(R,2)/r_square-pow(sin(phi_v),2))/(1+pow(sin(phi_v),2))));
      theta_m[0]  =  isnan(theta_m[0])||abs(theta_m[0])>pi/2 ? -pi/2:theta_m[0];
      theta_m[1]  =  asin(sqrt((pow(R,2)/r_square-pow(sin(phi_v),2))/(1+pow(sin(phi_v),2))));
      theta_m[1]  =  isnan(theta_m[1])||abs(theta_m[1])>pi/2 ? pi/2:theta_m[1];

      phi_m[0]    = -asin(sqrt((pow(R,2)/r_square-pow(sin(theta_v),2))/(1+pow(sin(theta_v),2))));
      phi_m[0]    = isnan(phi_m[0])||(abs(phi_m[0])>pi/2) ? -pi/2:phi_m[0];
      phi_m[1]    =  asin(sqrt((pow(R,2)/r_square-pow(sin(theta_v),2))/(1+pow(sin(theta_v),2))));
      phi_m[1]    = isnan(phi_m[1]) || (abs(phi_m[0])>pi/2) ? pi/2:phi_m[1];

      theta_min   = abs(theta_v - theta_m[0])< abs(theta_v - theta_m[1]) ? theta_m[0] : theta_m[1];
      phi_min     = abs(phi_v-phi_m[0]) < abs(phi_v-phi_m[1]) ? phi_m[0] : phi_m[1];

      if(n>=1000){
        if(abs(theta_v-theta_min) <= abs(phi_v-phi_min)) {
          data.theta_d =theta_min > 0 ? theta_v - theta_min +0.2 : theta_min - theta_v - 0.2;
          data.phi_d   =0;  

//          if(isnan(data.theta_d))   {data.theta_d=0;}
          std::cout<<"theta_d="<<data.theta_d<<std::endl;
        }
        else
        {
         data.theta_d = 0;
         data.phi_d   = phi_v>=phi_min ? phi_v - phi_min +0.1 : phi_min - phi_v - 0.1;
         std::cout<<"phi_d="<<data.phi_d<<std::endl;
        }
        n=0;
      }
      else
      {
        data.theta_d  =data.theta_d;
        data.phi_d    =data.phi_d;
        n++;
      }
    }
    else
    {
        data.theta_d  =0;
        data.phi_d    =0;
//        n=n>100 ? 0:n+1;
//      ROS_INFO("No obstacles");
    }

    pub.publish(data);
    loop.sleep();

  }


}

