#include<ros/ros.h>
#include<std_msgs/Float64MultiArray.h>
//#include"cpg_driver/the_function_of_cpg.h"
#include"cpg_driver_msg/cpg_param_change.h"
#include"cpg_driver_msg/differ.h"
#include"../include/cpg_driver_simulation/the_function_of_fish.h"
//#include"../../../devel/include/cpg_driver/cpg_param_change.h"
//#include"../../../devel/include/cpg_driver/differ.h"

CPG_unit                CPGs[9];
float 					CPG_differ_1[6]	=	{pi/6,0,pi/2,-pi/2,0,-pi/6};//for pectoral fins
float                   CPG_differ_2[2] =   {-pi/3,-pi/3};//for tail
float                   A[9]={0,0,0,0,0,0,0,0,0},X[9]={0,0,0,0,0,0,0,0,0};

void callback1(const cpg_driver_msg::cpg_param_change &par)
{
	X[0]=X[1]=X[2]=par.X_ca;
	//状态机判断//
	if(par.freq<=0.5)//低频运动模式
	{
		A[4]=0.5*asin(sin(2*par.A_pec_l)*cos(par.alfa_pec_l));
		A[3]=0.5*asin(sin(2*par.A_pec_l)*sin(par.alfa_pec_l));//左

		A[6]=0.5*asin(sin(2*par.A_pec_r)*cos(par.alfa_pec_r));
		A[7]=0.5*asin(sin(2*par.A_pec_r)*sin(par.alfa_pec_r));//右
		if(par.A_pec_l<0.01 && par.A_pec_l>-0.01)
			A[5]=0;
		else
		{
			A[5]=pi/4;
		}
		if(par.A_pec_r<0.01 && par.A_pec_r>-0.01)
			A[8]=0;
		else
		{
			A[8]=pi/4;
		}
		A[0]=A[1]=A[2]=0;
		if(par.alfa_pec_l>=0){//
			X[5]=(par.A_pec_l<0.01 && par.A_pec_l>-0.01) ? par.alfa_pec_l : par.alfa_pec_l-pi/4;
		}else{	
			X[5]=(par.A_pec_l<0.01 && par.A_pec_l>-0.01) ? par.alfa_pec_l : par.alfa_pec_l+pi/4;
		}
		if(par.alfa_pec_r>=0){//
			X[8]=(par.A_pec_r<0.01 && par.A_pec_r>-0.01) ? -par.alfa_pec_r : -par.alfa_pec_r-pi/4;
		}else{	
			X[8]=(par.A_pec_r<0.01 && par.A_pec_r>-0.01) ? -par.alfa_pec_r : -par.alfa_pec_r+pi/4;
		}
		
	}
	else if(par.freq>=2)//高频运动模式
	{
		A[3]=A[4]=A[5]=A[6]=A[7]=A[8]=0;
		A[0]=A[1]=par.A_ca;A[2]=1.2*A[0];
		
		X[5]=par.alfa_pec_l;
		X[8]=-par.alfa_pec_r;
		X[3]=0+0.5*asin(sin(2*par.A_pec_l)*cos(par.alfa_pec_l));
		X[4]=0+0.5*asin(sin(2*par.A_pec_l)*sin(par.alfa_pec_l));
		X[6]=0-0.5*asin(sin(2*par.A_pec_r)*cos(par.alfa_pec_r));
		X[7]=0-0.5*asin(sin(2*par.A_pec_r)*sin(par.alfa_pec_r));
	}
	else	//协同推进模式
	{
		A[4]=0.5*asin(sin(2*par.A_pec_l)*cos(par.alfa_pec_l));
		A[3]=0.5*asin(sin(2*par.A_pec_l)*sin(par.alfa_pec_l));//左

		A[6]=0.5*asin(sin(2*par.A_pec_r)*cos(par.alfa_pec_r));
		A[7]=0.5*asin(sin(2*par.A_pec_r)*sin(par.alfa_pec_r));//右
		if(par.A_pec_l<0.01 && par.A_pec_l>-0.01)
			A[5]=0;
		else
		{
			A[5]=pi/4;
		}
		if(par.A_pec_r<0.01 && par.A_pec_r>-0.01)
			A[8]=0;
		else
		{
			A[8]=pi/4;
		}
		A[0]=A[1]=par.A_ca;A[2]=1.2*A[0];
		if(par.alfa_pec_l>=0){//
			X[5]=(par.A_pec_l<0.01 && par.A_pec_l>-0.01) ? par.alfa_pec_l : par.alfa_pec_l-pi/4;
		}else{	
			X[5]=(par.A_pec_l<0.01 && par.A_pec_l>-0.01) ? par.alfa_pec_l : par.alfa_pec_l+pi/4;
		}
		if(par.alfa_pec_r>=0){//
			X[8]=(par.A_pec_r<0.01 && par.A_pec_r>-0.01) ? -par.alfa_pec_r : -par.alfa_pec_r+pi/4;
		}else{	
			X[8]=(par.A_pec_r<0.01 && par.A_pec_r>-0.01) ? -par.alfa_pec_r : -par.alfa_pec_r-pi/4;
		}
		
	}
	//cpg赋值//
	for(int i=0;i<9;i++)
	{
		CPGs[i].change('A',A[i]);
		CPGs[i].change('X',X[i]);
		CPGs[i].change('f',par.freq);
	}    
}


int main(int argc,char**argv)
{
    ros::init(argc,argv,"cpg_driver");


    ros::NodeHandle             nh;
    ros::Publisher              pub=nh.advertise<std_msgs::Float64MultiArray>("/naro/Naro_joint_position_controllers/command",10);
    ros::Subscriber             sub=nh.subscribe("module_change1",100,&callback1);
	std_msgs::Float64MultiArray data;
	

    ros::Rate                   loops(1/dt);
    while(ros::ok())
    {
        CPGs[0].computing();//CORE
        //tail//
        CPGs[1].computing(CPGs[2],CPGs[0],CPG_differ_2[1],-CPG_differ_2[0]);
        CPGs[2].computing(CPGs[2],CPGs[1],0              ,-CPG_differ_2[1]);
        //right//
        CPGs[3].computing(CPGs[4],CPGs[0],-CPG_differ_1[4],CPG_differ_1[3]);
        CPGs[4].computing(CPGs[5],CPGs[3],-CPG_differ_1[5],CPG_differ_1[4]);
        CPGs[5].computing(CPGs[5],CPGs[4],0              ,CPG_differ_1[5]);
        //left//
        CPGs[6].computing(CPGs[7],CPGs[0],-CPG_differ_1[1],CPG_differ_1[2]);
        CPGs[7].computing(CPGs[8],CPGs[6],-CPG_differ_1[0],CPG_differ_1[1]);
        CPGs[8].computing(CPGs[8],CPGs[7],0              ,CPG_differ_1[0]);
		

        for(int i=0;i<9;i++)
        {
            data.data.push_back(CPGs[i].out());
        }
        pub.publish(data);
		data.data.clear();
        ros::spinOnce();
        loops.sleep();
    }
}























