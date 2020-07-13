/*
    该程序用于无线串口通信与控制（接收端待测试）
*/

#include<ros/ros.h>
#include<serial/serial.h>
#include"./the_function_of_fish.h"
#include<cpg_driver_msg/cpg_param_change.h>
const uint8_t head[2]={0x73,0x73};
union data_trans
{
    /* data */
    uint8_t ch_data[4];
    float serial_data;
};

typedef struct param{
    uint8_t A_c[4];
    uint8_t X_c[4];
    uint8_t A_p_l[4];
    uint8_t alfa_p_l[4];
    uint8_t A_p_r[4];
    uint8_t alfa_p_r[4];
    uint8_t f[4];
} params;


const uint8_t tail[2]={0x64,0x64};
