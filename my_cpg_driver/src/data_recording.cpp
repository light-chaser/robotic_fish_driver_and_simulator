#include<ros/ros.h>
#include<fcntl.h>
#include<unistd.h>
#include<sys/stat.h>
#include<sensor_msgs/Imu.h>
#include"cpg_driver_msg/cpg_param_change.h"

int fd;

char                tab='\t';
char                ent='\n';
void callback(const cpg_driver_msg::data_record &datas){
    char ch_data[4][20];
    sprintf(ch_data[0],"%.3f",datas.a);
    sprintf(ch_data[1],"%.3f",datas.b);
    sprintf(ch_data[2],"%.3f",datas.c);
    sprintf(ch_data[3],"%.3f",datas.pitch);
    write(fd,(void*)&(ch_data[3]),strlen(ch_data[3]));
    write(fd,&(tab),sizeof(char));
    write(fd,(void*)&(ch_data[0]),strlen(ch_data[0]));
    write(fd,&(tab),sizeof(char));
    write(fd,(void*)&(ch_data[1]),strlen(ch_data[1]));
    write(fd,&(tab),sizeof(char));
    write(fd,(void*)&(ch_data[2]),strlen(ch_data[2]));
    write(fd,&(ent),sizeof(char));
}

int main(int argc,char**argv)
{
    ros::init(argc,argv,"data_recording");
    ros::NodeHandle h;
    
    ros::Subscriber sub=h.subscribe("data_records",10,callback);
    fd=open("~/record.txt",O_RDWR|O_CREAT|O_TRUNC,S_IRWXU);
    char *title="pitch\t\ta\t\tb\t\tc\t\t\n";

    write(fd,(void*)title,strlen(title));
    ros::spin();
    
}
