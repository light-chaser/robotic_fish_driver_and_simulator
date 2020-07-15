/*
 *   该程序用于无线串口通信与控制（发送端待测试）
 */

#include"../include/cpg_driver/wireless_serial.h"
#include"cpg_driver_msg/cpg_param_change.h"
serial::Serial ser;
param par;
data_trans A_c,A_p_l,A_p_r,X_c,alfa_p_l,alfa_p_r,f;


void serial_open(std::string &port);

void callback(const cpg_driver_msg::cpg_param_change &pa)
{
    A_c.serial_data         =pa.A_ca;
    A_p_l.serial_data       =pa.A_pec_l;
    A_p_r.serial_data       =pa.A_pec_r;
    alfa_p_l.serial_data    =pa.alfa_pec_l;
    alfa_p_r.serial_data    =pa.alfa_pec_r;
    f.serial_data           =pa.freq;
    X_c.serial_data         =pa.X_ca;
    memcpy(par.A_c,A_c.ch_data,sizeof(data_trans));
    memcpy(par.X_c,X_c.ch_data,sizeof(data_trans));
    memcpy(par.A_p_l,A_p_l.ch_data,sizeof(data_trans));
    memcpy(par.A_p_r,A_p_r.ch_data,sizeof(data_trans));
    memcpy(par.alfa_p_l,alfa_p_l.ch_data,sizeof(data_trans));
    memcpy(par.alfa_p_r,alfa_p_r.ch_data,sizeof(data_trans));
    memcpy(par.f,f.ch_data,sizeof(data_trans));
/*
    par.A_c                 =A_c.ch_data;
    par.A_p_l               =A_p_l.ch_data;
    par.A_p_r               =A_p_r.ch_data;
    par.alfa_p_l            =alfa_p_l.ch_data;
    par.alfa_p_r            =alfa_p_r.ch_data;
    par.f                   =f.ch_data;
    par.X_c                 =X_c.ch_data;
 */   
    ser.write(head,sizeof(head));
    ser.write((uint8_t*)&par,sizeof(par));
    ser.write(tail,sizeof(tail));
}



int main(int argc,char**argv)
{
    ros::init(argc,argv,"wireless_controller");
    ros::NodeHandle n;
    ros::Subscriber sub=n.subscribe("module_change1",10,callback);    
    std::string     port="/dev/ttyUSB0";
    ros::Rate       loop(1);
    serial_open(port);
//    ros::AsyncSpinner spinner(2);
//    spinner.start();
    while(ros::ok()){
        ros::spinOnce();      
        loop.sleep();
    }
    
}
void serial_open(std::string &port){
	try 
    { 
    //设置串口属性，并打开串口 
        ser.setPort(port); 
        ser.setBaudrate(9600); 
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
