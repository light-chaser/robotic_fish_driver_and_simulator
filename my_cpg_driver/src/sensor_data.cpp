/*该程序用于读取所有传感器数据*/

#include<sensor_msgs/Imu.h>
#include<ros/ros.h>
#include "../include/cpg_driver/JY901.h"
#include"../../../devel/include/cpg_driver_msg/buffer.h"
#include<sensor_msgs/FluidPressure.h>
#define g 9.8;

CJY901 JY901;
sensor_msgs::Imu imu_data;
sensor_msgs::FluidPressure pressure1,pressure2;
void sensor_callback(const cpg_driver_msg::buffer &oringin_data)
{       
        uint8_t buf[100];
        memcpy(buf,oringin_data.buffer.data(),sizeof(uint8_t)*100);        
        JY901.CopeSerialData((char*)buf,100);

        imu_data.header.stamp=ros::Time::now();
        imu_data.header.frame_id="base";

/*		imu_data.linear_acceleration.x=(double)(JY901.stcAcc.a[0]<<8|(JY901.stcAcc.a[0]&0xff00))/32768/16*g;
		imu_data.linear_acceleration.y=(double)(JY901.stcAcc.a[1]<<8|(JY901.stcAcc.a[1]&0xff00))/32768/16*g;
		imu_data.linear_acceleration.z=(double)(JY901.stcAcc.a[2]<<8|(JY901.stcAcc.a[2]&0xff00))/32768/16*g;

        imu_data.angular_velocity.x=(double)(JY901.stcGyro.w[0]<<8|(JY901.stcGyro.w[0]&0xff00))/32768*2000;
        imu_data.angular_velocity.y=(double)(JY901.stcGyro.w[1]<<8|(JY901.stcGyro.w[1]&0xff00))/32768*2000;
        imu_data.angular_velocity.z=(double)(JY901.stcGyro.w[2]<<8|(JY901.stcGyro.w[2]&0xff00))/32768*2000;
        
		imu_data.orientation.x=(double)(JY901.stcQuartern.sQuartern[2]<<8|(JY901.stcQuartern.sQuartern[2]&0xff00))/32768;
        imu_data.orientation.y=(double)-(JY901.stcQuartern.sQuartern[1]<<8|(JY901.stcQuartern.sQuartern[1]&0xff00))/32768;
        imu_data.orientation.z=(double)-(JY901.stcQuartern.sQuartern[0]<<8|(JY901.stcQuartern.sQuartern[0]&0xff00))/32768;
        imu_data.orientation.w=(double)(JY901.stcQuartern.sQuartern[3]<<8|(JY901.stcQuartern.sQuartern[3]&0xff00))/32768;
*/
        imu_data.linear_acceleration.x=(double)(JY901.stcAcc.a[0])/32768*16*g;
        imu_data.linear_acceleration.y=(double)(JY901.stcAcc.a[1])/32768*16*g;
        imu_data.linear_acceleration.z=(double)(JY901.stcAcc.a[2])/32768*16*g;

        imu_data.angular_velocity.x=(double)(JY901.stcGyro.w[0])/32768*2000;
        imu_data.angular_velocity.y=(double)(JY901.stcGyro.w[1])/32768*2000;
        imu_data.angular_velocity.z=(double)(JY901.stcGyro.w[2])/32768*2000;

        imu_data.orientation.x=(double)(JY901.stcQuartern.sQuartern[2])/32768;
        imu_data.orientation.y=(double)-(JY901.stcQuartern.sQuartern[1])/32768;
        imu_data.orientation.z=(double)-(JY901.stcQuartern.sQuartern[0])/32768;
        imu_data.orientation.w=(double)(JY901.stcQuartern.sQuartern[3])/32768;

        pressure1.fluid_pressure=(double)(JY901.stcPress.Pressure1)/10;
        pressure2.fluid_pressure=(double)(JY901.stcPress.Pressure2)/10;
}

int main(int argc,char**argv)
{
    ros::init(argc,argv,"imu_data");
    ros::NodeHandle n;
    ros::Publisher pub   =  n.advertise<sensor_msgs::Imu>("Imu_data",10);
    ros::Publisher pub2 =  n.advertise<sensor_msgs::FluidPressure>("pressure1",10);
    ros::Publisher pub3 =  n.advertise<sensor_msgs::FluidPressure>("pressure2",10);
    ros::Subscriber sub=n.subscribe("origin_data",100,&sensor_callback);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    while (ros::ok())
    {
        pub.publish(imu_data);
        pub2.publish(pressure1);
        pub3.publish(pressure2);
    }
    
}
