#include<sensor_msgs/Joy.h>
#include<ros/ros.h>
#include"../../../devel/include/cpg_driver_msg/cpg_param_change.h"
#define p_max 0.5;
#ifndef pi
#define pi 3.14
#endif
#ifndef dt
#define dt 1
#endif
cpg_driver_msg::cpg_param_change param;
bool a=false;

int switches=0;
void joy_callback1(const sensor_msgs::Joy &joy)
{
    if(joy.buttons[0]==1){
        switches+=1;
        if(switches>3){switches=1;}
    }
    switch (switches)
    {
    case  1://胸鳍推进
        param.freq                =1.5*(1-joy.axes[2]);
        param.A_pec_l        =(joy.axes[1]+joy.axes[0])*p_max;
        param.A_pec_r        =(joy.axes[1]-joy.axes[0])*p_max;
        param.alfa_pec_l    =pi/2*(joy.axes[4]);
        param.alfa_pec_r   =param.alfa_pec_l;
        param.A_ca              =0;
        param.X_ca              =joy.axes[0]*0.4;
        a=true;
        break;
    case  2://尾部推进
        param.freq=1.5*(1-joy.axes[2]);
        param.A_pec_l=0;
        param.A_pec_r=0;
        param.alfa_pec_l=pi/2*(joy.axes[4]);
        param.alfa_pec_r=param.alfa_pec_l;
        param.A_ca=joy.axes[1]*0.5;
        param.X_ca=joy.axes[0]*0.4;
        a=true;
        break;
    case  3://胸 / 尾鳍协同推进
        param.freq=1.5*(1-joy.axes[2]);
        param.A_pec_l=(joy.axes[1]+joy.axes[0])*p_max;
        param.A_pec_r=(joy.axes[1]-joy.axes[0])*p_max;
        param.alfa_pec_l=pi/2*(joy.axes[4]);
        param.alfa_pec_r=param.alfa_pec_l;
        param.A_ca=joy.axes[1]*0.5;
        param.X_ca=joy.axes[0]*0.4;
        a=true;
        break;
    default:
        a=false;
        break;
    }
}

int main(int argc,char**argv)
{
    ros::init(argc,argv,"joy_control");
    ros::NodeHandle n;
    ros::Subscriber joy1=n.subscribe("joy",10,joy_callback1);
//    ros::Subscriber joy2=n.subscribe("joy",10,joy_callback2);
    ros::Publisher pub=n.advertise<cpg_driver_msg::cpg_param_change>("module_change1",10);
    ros::AsyncSpinner spinner(3);
    ros::Rate loop(1/dt);
    spinner.start();
    while(ros::ok())
    {
        if(a){
            pub.publish(param);
            a=false;
        }
            
//	    ros::spinOnce();
        loop.sleep();
    }
}
