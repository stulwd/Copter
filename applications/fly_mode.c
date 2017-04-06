#include "fly_mode.h"
#include "rc.h"

u8 mode_state,mode_state_old;
void mode_check(float *ch_in)
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
	
	//===========   ===========
	mode_state_old = mode_state; //历史模式
}
