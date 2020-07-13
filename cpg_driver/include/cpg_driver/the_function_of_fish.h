/*本函数库用于存放机器鱼的常用控制架构,*
 *以及控制算法，希望对接下来的研究有所帮助，*
 *使用GPL开源协议，放心使用*
 */
/*作者：张宇杰*/

#define dt 0.001
#define pi 3.14

#include<math.h>
#include<iostream>
using namespace std;
struct CPGs    //CPG振荡器数据结构
{
    float A;//参考幅值
    float X;//参考中位
    float freq;//振荡频率
    float time_rate;//时间比率
    float a;    //幅度
    float x;    //实际中位
    float faiz; //振荡相位
    float theta;//振荡输出
};
float sgn(float m){//信号函数
    if(m>=0)
        return 1;
    else
    {
        return -1;
    }
}
class CPG_unit
{
private:
    CPGs CPG_data;    
public:

    CPG_unit(/* args */);
    ~CPG_unit();
    void computing(CPG_unit fore_m,CPG_unit before_m,float fore, float before);
    void computing();
    void change(char c,float num);
    void print_data();
	float out();
};

CPG_unit::CPG_unit(/* args */)//所有CPG的初始值
{
    CPG_data.A=0;
    CPG_data.X=pi/2;
    CPG_data.freq=1;
    CPG_data.a=0;
    CPG_data.x=0;
    CPG_data.faiz=0;
    CPG_data.theta=0;
    CPG_data.time_rate=0;
}
CPG_unit::~CPG_unit(){}

void CPG_unit::print_data()
{
    std::cout<<"A="<<CPG_data.A<<endl
        <<"X="<<CPG_data.X<<endl
        <<"freq="<<CPG_data.freq<<endl<<endl
        <<"a="<<CPG_data.a<<endl
        <<"faiz="<<CPG_data.faiz<<endl
        <<"x="<<CPG_data.x<<endl
        <<"theta="<<CPG_data.theta<<endl<<endl;
}

void CPG_unit::computing(CPG_unit fore_m,CPG_unit before_m,float fore,float before)//迭代计算（两两耦合情形）
{
    CPG_data.a+=dt*10*(CPG_data.A-CPG_data.a);
    CPG_data.x+=dt*10*(CPG_data.X-CPG_data.x);
    CPG_data.faiz+=dt*(2*pi*CPG_data.freq*(1+CPG_data.time_rate*sgn(sin(CPG_data.faiz)))
                       +5*(fore_m.CPG_data.faiz-CPG_data.faiz-fore)
                       +5*(before_m.CPG_data.faiz-CPG_data.faiz-before));
    CPG_data.theta=CPG_data.x+CPG_data.a*cos(CPG_data.faiz);
}
void CPG_unit::computing()//迭代计算（单独运行情况）
{
    CPG_data.a+=dt*1.2*(CPG_data.A-CPG_data.a);
    CPG_data.x+=dt*1.2*(CPG_data.X-CPG_data.x);
    CPG_data.faiz+=dt*(2*pi*CPG_data.freq*(1+CPG_data.time_rate*sgn(sin(CPG_data.faiz))));
    CPG_data.theta=CPG_data.x+CPG_data.a*cos(CPG_data.faiz);
}
void CPG_unit::change(char c,float num)//参数修改
{
    switch (c)
    {
    case 'A':
        {
            CPG_data.A=num;
            break;
        }
    case 'X':
        {
            CPG_data.X=num;
            break;
        }
    case 'f':
        {
            CPG_data.freq=num;
            break;
        }
    case 'R':
        {
            CPG_data.time_rate=num;
            break;
        }

    default:
        break;
    }
}
float CPG_unit::out(){return CPG_data.theta;}

