/* 包含头文件 ----------------------------------------------------------------*/
#include "RemoteControl/et16s_rf209.h"
#include "string.h"
#include "stdlib.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
WBUS_info_t WBUS_info;
WBUS_info_t last_WBUS_info;
uint8_t WBUS_buffer[18];
/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/
static void Frame_Loss_protection(WBUS_info_t *WBUS);
/* 函数体 --------------------------------------------------------------------*/
void RC_WBUSDataParser(WBUS_info_t *WBUS, uint8_t *buf, uint16_t len)
{
    if(buf == NULL)
    {
        return;
    }
    if(buf[0]!= StartByte)
    {
        return;
    }
    else  if(buf[0]== StartByte)
    {
		WBUS->Ch1  = ((buf[1]    |buf[2]<<8)                 & 0x07FF)-RC_ET_STICK_OFFSET;
		WBUS->Ch2  = ((buf[2]>>3 |buf[3]<<5)                 & 0x07FF)-RC_ET_STICK_OFFSET;
		WBUS->Ch3 = ((buf[3]>>6 |buf[4]<<2 |buf[5]<<10)  & 0x07FF)-RC_ET_STICK_OFFSET;
		WBUS->Ch4  = ((buf[5]>>1 |buf[6]<<7)                 & 0x07FF)-RC_ET_STICK_OFFSET-8;
		WBUS->Ch5  = ((buf[6]>>4 |buf[7]<<4)                 & 0x07FF);
		WBUS->Ch6  = ((buf[7]>>7 |buf[8]<<1 |buf[9]<<9)   & 0x07FF);
		WBUS->Ch7 = ((buf[9]>>2 |buf[10]<<6)                & 0x07FF);
		WBUS->Ch8  = ((buf[10]>>5|buf[11]<<3)                & 0x07FF);
		WBUS->Ch9  = ((buf[12]   |buf[13]<<8)                & 0x07FF);
		WBUS->Ch10  = ((buf[13]>>3|buf[14]<<5)                & 0x07FF);
		WBUS->Ch11 = ((buf[14]>>6|buf[15]<<2|buf[16]<<10) & 0x07FF);
		WBUS->Ch12 = ((buf[16]>>1|buf[17]<<7)                & 0x07FF);
		WBUS->Ch13 = ((buf[17]>>4|buf[18]<<4)                & 0x07FF);
		WBUS->Ch14 = ((buf[18]>>7|buf[19]<<1|buf[20]<<9)  & 0x07FF);
		WBUS->Ch15 = ((buf[20]>>2|buf[21]<<6)                & 0x07FF);
		WBUS->Ch16 = ((buf[21]>>5|buf[22]<<3)                & 0x07FF);
		WBUS->Flag=buf[23];
		WBUS->End=buf[24];
        
		// if (time>200)
		// {
        Frame_Loss_protection(&WBUS_info);
		// }

        if(WBUS->Ch3<5&&WBUS->Ch3>-5)
        WBUS->Ch3=0;

        if(WBUS->Ch4<5&&WBUS->Ch4>-5)
        {
            WBUS->Ch4=0;/* code */
        }
        /*通道映射区*/
        WBUS->toggle_switch.SF=WBUS->Ch5;
				WBUS->toggle_switch.SH=WBUS->Ch6;
				WBUS->toggle_switch.SG=WBUS->Ch7;
				WBUS->toggle_switch.SE=WBUS->Ch8;
        WBUS->toggle_switch.LD=WBUS->Ch9;
				WBUS->toggle_switch.RD=WBUS->Ch10;
        WBUS->toggle_switch.SA=WBUS->Ch11;
        WBUS->toggle_switch.SB=WBUS->Ch12;
        WBUS->toggle_switch.SC=WBUS->Ch13;
        WBUS->toggle_switch.SD=WBUS->Ch14;
	/*接收机原始数据发送缓冲区*/	
	for(uint8_t i=0;i<24;i++)	
	WBUS_buffer[i]= buf[i];// buf[i];
    last_WBUS_info=WBUS_info;
    }
}

void RC_gimbal_DataParser(WBUS_info_t *WBUS, uint8_t *buf, uint16_t len)
{
	if(buf == NULL)
    {
        return;
    }
    if(buf[0]!= StartByte)
    {
        return;
    }
    else  if(buf[0]== StartByte)
    {
		WBUS->Ch1  = ((buf[1]    |buf[2]<<8)                 & 0x07FF)-RC_ET_STICK_OFFSET;
		WBUS->Ch2  = ((buf[2]>>3 |buf[3]<<5)                 & 0x07FF)-RC_ET_STICK_OFFSET;
		WBUS->Ch3 = ((buf[3]>>6 |buf[4]<<2 |buf[5]<<10)  & 0x07FF)-RC_ET_STICK_OFFSET;
		WBUS->Ch4= ((buf[5]>>1 |buf[6]<<7)                 & 0x07FF)-RC_ET_STICK_OFFSET;
		WBUS->Ch5  = ((buf[6]>>4 |buf[7]<<4)                 & 0x07FF);
		WBUS->Ch6  = ((buf[7]>>7 |buf[8]<<1 |buf[9]<<9)   & 0x07FF);
		WBUS->Ch7 = ((buf[9]>>2 |buf[10]<<6)                & 0x07FF);
		WBUS->Ch8  = ((buf[10]>>5|buf[11]<<3)                & 0x07FF);
		WBUS->Ch9  = ((buf[12]   |buf[13]<<8)                & 0x07FF);
		WBUS->Ch10  = ((buf[13]>>3|buf[14]<<5)                & 0x07FF);
		WBUS->Ch11 = ((buf[14]>>6|buf[15]<<2|buf[16]<<10) & 0x07FF);
		WBUS->Ch12 = ((buf[16]>>1|buf[17]<<7)                & 0x07FF);
		WBUS->Ch13 = ((buf[17]>>4|buf[18]<<4)                & 0x07FF);
		WBUS->Ch14 = ((buf[18]>>7|buf[19]<<1|buf[20]<<9)  & 0x07FF);
		WBUS->Ch15 = ((buf[20]>>2|buf[21]<<6)                & 0x07FF);
		WBUS->Ch16 = ((buf[21]>>5|buf[22]<<3)                & 0x07FF);
		WBUS->Flag=buf[23];
	}

        Frame_Loss_protection(&WBUS_info);


        if(WBUS->Ch3<5&&WBUS->Ch3>-5)
        WBUS->Ch3=0;

        if(WBUS->Ch4<5&&WBUS->Ch4>-5)
        {
            WBUS->Ch4=0;/* code */
        }

        WBUS->toggle_switch.SF=WBUS->Ch5;
				WBUS->toggle_switch.SH=WBUS->Ch6;
				WBUS->toggle_switch.SG=WBUS->Ch7;
				WBUS->toggle_switch.SE=WBUS->Ch8;

        WBUS->toggle_switch.LD=WBUS->Ch9;
		    WBUS->toggle_switch.RD=WBUS->Ch10;

        WBUS->toggle_switch.SA=WBUS->Ch11;
        WBUS->toggle_switch.SB=WBUS->Ch12;
        WBUS->toggle_switch.SC=WBUS->Ch13;
        WBUS->toggle_switch.SD=WBUS->Ch14;

        last_WBUS_info=WBUS_info;
	
}



void RC_ET_SwitchAction(WBUS_Switch_t *sw, uint16_t value)
{
    /* 最新状态值  */
    sw->switch_value = value;

    /* 取最新值和上一次值  */
	if(sw->last_switch_value==REMOTE_ET_SWITCH_VALUE_DOWN&&sw->switch_value==REMOTE_ET_SWITCH_VALUE_UP)
    sw->switch_state = Rising_edge;
	else 
    if(sw->last_switch_value==REMOTE_ET_SWITCH_VALUE_UP&&sw->switch_value==REMOTE_ET_SWITCH_VALUE_DOWN)
    sw->switch_state = Falling_edge;

    /* 更新上一次值 */
    sw->last_switch_value = sw->switch_value;
}


WBUS_info_t* WBUS_GetDataPointer(void)
{
    return &WBUS_info;
}   

static void Frame_Loss_protection(WBUS_info_t *WBUS)
{
    // 检查前四个通道和9、10通道
    for (int i = 0; i < 4; i++) {
        int16_t *current_channel = &WBUS->Ch1 + i;
        int16_t *last_channel = &last_WBUS_info.Ch1 + i;

        if (abs(*current_channel - *last_channel) >= 512) {
            *current_channel = *last_channel;
        }
    }

    // 检查其他通道
    for (int i = 4; i < 16; i++) {
        if (i == 8 || i == 9) continue; // 跳过第9、10通道

        int16_t *current_channel = &WBUS->Ch1 + i;
        int16_t *last_channel = &last_WBUS_info.Ch1 + i;

        if (*current_channel != 352 && *current_channel != 1024 && *current_channel != 1695) {
            *current_channel = *last_channel;
        }
    }

}
