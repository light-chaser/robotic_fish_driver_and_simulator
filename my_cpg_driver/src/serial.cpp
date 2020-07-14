#include<ros/ros.h>
#include<serial/serial.h>
#include<cpg_driver_msg/buffer.h>
#include<cpg_driver_msg/gait.h>
#include<iostream>

char 	                head[3]	="ss";//包头
char 	                tail[3]	="ee";//包尾
std::string 		port	=	"/dev/ttyUSB0";//uart串口端口

class serial_node{
private:
    serial::Serial ser;
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    cpg_driver_msg::buffer buf;

public:
    serial_node();
    void gait_callback(const cpg_driver_msg::gait &gait);
    void run();
    void serial_open(std::string &port);
};


serial_node::serial_node(){
     serial_open(port);//打开串口
     sub=nh.subscribe("gait",10,  &serial_node::gait_callback,this);//订阅步态
     pub=nh.advertise<cpg_driver_msg::buffer>("origin_data",10);//发布传感器原始数据
 }

void serial_node::gait_callback(const cpg_driver_msg::gait &gait)
{
    ser.write((uint8_t*)head,sizeof(head)-1);
    ser.write((uint8_t*)gait.buffer.begin(),gait.buffer.size()*sizeof(int16_t));//写入步态数据
    ser.write((uint8_t*)tail,sizeof(tail)-1);
}


void serial_node::run(){
    ros::Rate loop_rate(50);
    while(ros::ok()){
        ser.read(buf.buffer.begin(),100);//读取串口数据
        pub.publish(buf);//发布串口数据
        loop_rate.sleep();
    }
}

void serial_node::serial_open(std::string &port){
	try 
    { 
    //设置串口属性，并打开串口 
        ser.setPort(port); 
        ser.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        exit(-1); 
    } 
  
    //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        exit(-1); 
    } 
}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"serial_node");
    serial_node node;
    ros::AsyncSpinner spinner(1); // Use 1 threads
    spinner.start();
    node.run();
}
