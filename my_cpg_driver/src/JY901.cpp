#include <stdio.h>
#include "../include/cpg_driver/JY901.h"
#include "string.h"

CJY901 ::CJY901 ()
{
}

void CJY901 ::CopeSerialData(char ucData[],unsigned short usLength)
{
	static unsigned char chrTemp[2000];
	static unsigned char ucRxCnt = 0;	
	static unsigned short usRxLength = 0;
	static short i=0;

    memcpy(chrTemp,ucData,usLength);
	usRxLength += usLength;
    while (usRxLength >= 11)
    {
        if (chrTemp[0] != 0x45)
        {
			usRxLength--;
			memcpy(&chrTemp[0],&chrTemp[1],usRxLength);                        
            continue;
        }
		switch(chrTemp[1])
		{
//			case 0x40:	memcpy(&stcTime,&chrTemp[2],8);break;
			case 0x41:	memcpy(&stcAcc,&chrTemp[2],8);break;//加速度
			case 0x42:	memcpy(&stcGyro,&chrTemp[2],8);break;//角速度
			case 0x43:	memcpy(&stcAngle,&chrTemp[2],8);break;//角度（欧拉角）
			case 0x44:	memcpy(&stcQuartern,&chrTemp[2],8);break;  //四元数？？
			case 0x45:  memcpy(&stcPress,&chrTemp[2],8);break;//压强
		}
		usRxLength -= 11;
		memcpy(&chrTemp[0],&chrTemp[11],usRxLength);
		i+=1;
		if(i>=5){
			i=0;
			break;
		}	                     
    }
}
