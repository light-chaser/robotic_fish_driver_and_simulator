/*
    该程序用于无线串口通信与控制（接收端待测试）
*/
#include"../include/cpg_driver/wireless_serial.h"
#include"cpg_driver_msg/cpg_param_change.h"
serial::Serial ser;
void serial_open(std::string &port);
params par;
data_trans A_c,A_p_l,A_p_r,X_c,alfa_p_l,alfa_p_r,f;

void save_data(uint8_t* buf)
{
    uint8_t temp[60];
    memcpy(temp,buf,60);
    int length=sizeof(temp);
    while(length>=32)
    {
        if(temp[0]!=0x73){
            length--;
            memcpy(&temp[0],&temp[1],length);
        }
        else
        if(temp[1]==0x73)
        {
            cout<<"got the data.\n";
            memcpy((void*)&par,&temp[2],sizeof(params));
            length-=32;
            memcpy(&temp[0],&temp[31],length);
            memcpy(A_c.ch_data,par.A_c,sizeof(data_trans));
            memcpy(X_c.ch_data,par.X_c,sizeof(data_trans));
            memcpy(A_p_l.ch_data,par.A_p_l,sizeof(data_trans));
            memcpy(A_p_r.ch_data,par.A_p_r,sizeof(data_trans));
            memcpy(alfa_p_l.ch_data,par.alfa_p_l,sizeof(data_trans));
            memcpy(alfa_p_r.ch_data,par.alfa_p_r,sizeof(data_trans));
            memcpy(f.ch_data,par.f,sizeof(data_trans));
        }
    }
}


int main(int argc,char**argv)
{
    ros::init(argc,argv,"wireless_controlled");
    ros::NodeHandle     n;
    uint8_t             buf[60];//暂存数据
    std::string         port="/dev/ttyUSB0";
    cpg_driver_msg::cpg_param_change pars;
    serial_open(port);
    ros::Publisher pub1=n.advertise<cpg_driver_msg::cpg_param_change>("module_change",10);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    while(ros::ok())
    {
        int i;
        if(i=ser.read(buf,sizeof(buf)))
        {
            cout<<i<<endl;
            save_data(buf);
            pars.A_ca           =A_c.serial_data;
            pars.A_pec_l        =A_p_l.serial_data;
            pars.A_pec_r        =A_p_r.serial_data;
            pars.alfa_pec_l     =alfa_p_l.serial_data;
            pars.alfa_pec_r     =alfa_p_r.serial_data;
            pars.X_ca           =X_c.serial_data;
            pars.freq           =f.serial_data;
            pub1.publish(pars);
        }
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
