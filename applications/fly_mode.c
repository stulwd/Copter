#include "fly_mode.h"
#include "rc.h"

u8 mode_value[10];
u8 mode_state,mode_state_old;
void mode_check(float *ch_in,u8 *mode_value)
{
	//根据AUX1通道（第5通道）的数值切换飞行模式
	if(*(ch_in+AUX1) <-200)			//最低
	{
		mode_state = 0;	//手动油门
	}
	else if(*(ch_in+AUX1) >200)		//中间
	{
		mode_state = 2;	//超声波+气压计融合
	}
	else							//最高
	{
		mode_state = 1;	//气压计定高
	}
	
	//=========== GPS、气压定高 ===========
	if(mode_state == 0 )
	{
		*(mode_value+BARO) = 0;
	}
	else
	{
		*(mode_value+BARO) = 1;
	}
	
//	//=========== 返航模式 ===========
//	if(fly_ready )
//	{
//		if(( mode_state == 2 && mode_state_old != 2) || rc_lose == 1)
//		{

//			*(mode_value+BACK_HOME) = 1;
//			

//		}
//		else if(mode_state != 2)
//		{
//			*(mode_value+BACK_HOME) = 0;
//		}
//	}
//	else
//	{
//		*(mode_value+BACK_HOME) = 0;
//	}
	
	
 
	//===========   ===========
	mode_state_old = mode_state; //历史模式
}
