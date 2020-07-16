
 /*
 此为机器鱼驱动程序模块，为所有其他模块的核心组件，用于控制机器鱼

 CPG网络模型：
 *
 *(3)<----(2)     (5)---->(6)
           |       |
 *	  (1)	  (4)
    (单向) \	  / (单向)
	      (7)
 */
#include <ros/ros.h>
#include <serial/serial.h>
#include <cmath>

#include <cpg_driver_msg/cpg_param_change.h>
//#include "../../../devel/include/cpg_driver/cpg_param_change.h"
#include <cpg_driver_msg/differ.h>
#include "../include/cpg_driver/the_function_of_fish.h"
#include<std_msgs/UInt8MultiArray.h>
#include"cpg_driver_msg/gait.h"
#include<pthread.h>

using namespace std;

CPG_unit 				CPGs[7];//CPG单元类
//CPG相位差，其中0~1为右胸鳍两两之间相位差，2~3为左胸鳍两两之间相位差，4为右胸鳍与尾鳍间相位差，5为左胸鳍和鱼鳍间相位差。//
float 							CPG_differ[6]	=	{pi/3,0,0,pi,0,-pi/3};

//此处用于写回调函数//
float A[7],X[7];

void param_change_callback(const cpg_driver_msg::cpg_param_change &par){//改参数
	for(int i=0;i<6;i++)
	{
		X[i]=pi/2;
	}
	//状态机判断//
	if(par.freq<=0.5)//低频运动模式
	{
		A[0]=0.5*asin(sin(2*par.A_pec_l)*cos(par.alfa_pec_l));
		A[1]=0.5*asin(sin(2*par.A_pec_l)*sin(par.alfa_pec_l));//左

		A[3]=0.5*asin(sin(2*par.A_pec_r)*cos(par.alfa_pec_r));
		A[4]=0.5*asin(sin(2*par.A_pec_r)*sin(par.alfa_pec_r));//右
		
		A[6]=0;
		if(par.alfa_pec_l>=0){//
			A[2]=pi/4;
			X[2]=par.alfa_pec_l+pi/4;
		}else{	
			X[2]=par.alfa_pec_l+3*pi/4;
		}
		if(par.alfa_pec_r>=0){//
			A[5]=pi/4;
			X[5]=0.75*pi-par.alfa_pec_r;
		}else{	
			X[5]=0.25*pi-par.alfa_pec_r;
		}
		X[6]=par.X_ca+pi/2+0.1;
	}
	else if(par.freq>=2.5)//高频运动模式
	{
		A[0]=A[1]=A[2]=A[3]=A[4]=A[5]=0;
		A[6]=par.A_ca;
		X[6]=par.X_ca+pi/2+0.1;
		X[2]=par.alfa_pec_l;
		X[5]=pi-par.alfa_pec_r;
		X[0]=pi/2+0.5*asin(sin(2*par.A_pec_l)*cos(par.alfa_pec_l));
		X[1]=pi/2+0.5*asin(sin(2*par.A_pec_l)*sin(par.alfa_pec_l));
		X[3]=pi/2-0.5*asin(sin(2*par.A_pec_r)*cos(par.alfa_pec_r));
		X[4]=pi/2-0.5*asin(sin(2*par.A_pec_r)*sin(par.alfa_pec_r));
	}
	else	//协同推进模式
	{
		A[0]=0.5*asin(sin(2*par.A_pec_l)*cos(par.alfa_pec_l));
		A[1]=0.5*asin(sin(2*par.A_pec_l)*sin(par.alfa_pec_l));//左

		A[3]=0.5*asin(sin(2*par.A_pec_r)*cos(par.alfa_pec_r));
		A[4]=0.5*asin(sin(2*par.A_pec_r)*sin(par.alfa_pec_r));//右
		
		if( abs(par.A_pec_l)>0.01){
			A[2]=pi/4;
			if(par.alfa_pec_l>=0){//
				X[2]=par.alfa_pec_l+pi/4;
			}else{	
				X[2]=par.alfa_pec_l+3*pi/4;
			}
		}else{
			X[2]=pi/2+par.alfa_pec_l;
		}
		
		if(abs(par.A_pec_r)>0.01){
			A[5]=pi/4;
			if(par.alfa_pec_r>=0){//
				X[5]=0.75*pi-par.alfa_pec_r;
			}else{	
				X[5]=0.25*pi-par.alfa_pec_r;
			}
		}else{
			X[5]=pi/2+par.alfa_pec_r;
		}
		
		A[6]=par.A_ca;//尾部
		X[6]=par.X_ca+pi/2+0.1;
		
	}
	//cpg赋值//
	for(int i=0;i<7;i++)
	{
		CPGs[i].change('A',A[i]);
		CPGs[i].change('X',X[i]);
		CPGs[i].change('f',par.freq);
	}
}

void differ_change_callback(const cpg_driver_msg::differ &dif){//改相位差
	if(dif.n>6)
	ROS_INFO("It has only 6 differ,RECHANGE!!");
	else
	{
		CPG_differ[dif.n]=dif.value;
	}
}

int main(int argc,char**argv){
	ros::init(argc,argv,"CPG_node");//ROS节点初始化二连
	ros::NodeHandle		nh;
	cpg_driver_msg::gait tsm;
	ros::Rate 			loop_rate(1/dt);
	ros::Subscriber 	sub		=	nh.subscribe("module_change",50,&param_change_callback);//改步态
	ros::Subscriber 	sub2	=	nh.subscribe("differ_change",50,&differ_change_callback);//改相位
	ros::Publisher 		gait_pub = 	nh.advertise<cpg_driver_msg::gait>("gait", 10);
	ROS_INFO("fish start。");
	
	ros::AsyncSpinner spiner(2);
	spiner.start();
	while(ros::ok()){
		
		CPGs[6].computing();//尾巴
		//右胸鳍//
		CPGs[0].computing(CPGs[1],CPGs[6],CPG_differ[1],CPG_differ[2]);
		CPGs[1].computing(CPGs[2],CPGs[0],CPG_differ[0],CPG_differ[1]);
		CPGs[2].computing(CPGs[2],CPGs[1],0,CPG_differ[1]);
		//左胸鳍//
		CPGs[3].computing(CPGs[6],CPGs[4],CPG_differ[3],CPG_differ[4]);
		CPGs[4].computing(CPGs[3],CPGs[5],CPG_differ[4],-CPG_differ[5]);
		CPGs[5].computing(CPGs[5],CPGs[4],0,CPG_differ[5]);
//		CPGs[2].print_data();//观测用



		for(int i=0;i<7;i++)
			tsm.buffer.c_array()[i]=20000*CPGs[i].out()/pi+5000;
		tsm.buffer.c_array()[7]=tsm.buffer.c_array()[6];
		

/*将步态数据发送至串口节点*/
		gait_pub.publish(tsm);
		loop_rate.sleep();
//		ros::spinOnce();

	}	
	return 0;

}



