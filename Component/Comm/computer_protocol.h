#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "struct_typedef.h"
#include "comm_protocol.h"
#include "decision_task.h"
#include "referee_system.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
    VISION_DATA_CMD_ID          = 0x0001,
		NAV_DATA_CMD_ID             = 0x0002,
} COMPUTER_CMD_ID_e;

typedef enum
{
    Gray = 0,
    Red = 1,
    Blue = 2,
    AllColor = Red|Blue
} EnemyColor_e;

typedef enum
{
    VISION_TRACK_LOSS       = 0x00,
    VISION_TRACK            = 0x01,
} VisionState_e;

//tx
#pragma pack(push,1)
typedef struct
{
   	uint8_t 	data_head;     
	uint8_t 	enemy_color;
	fp32    	yaw_relative_angle;
	fp32    	pitch_relative_angle;
} Comm_RobotInfo_t;

typedef struct
{
	ext_game_state_t* game_state;
	ext_game_robot_HP_t* game_robot_HP_t;
	ext_bullet_remaining_t* bullet_remaining_t;
	fp32 x;
//	uint16_t bullet_remaining_num_17mm;
//	uint16_t ally_sentry_HP;
//	uint16_t ally_infantry_HP;
//	uint16_t ally_hero_HP;
//	uint16_t enemy_sentry_HP;
//	uint16_t enemy_infantry_HP;
//	uint16_t enemy_hero_HP;
} Comm_DecisionInfo_t;

//rx

typedef struct
{
    uint8_t    data_head;    						 //头帧 默认0xAA
	uint8_t    pitch[4];							 /* Pitch角度值 */
	uint8_t    yaw[4];								 /* Yaw角度值 */
	uint8_t    palstance[4];						 /* Yaw角速度值 */
	uint8_t	   can_shoot[2];       			 		 /* 自动打弹标志位 */
	uint8_t	   move_state[2];       	 	 		 /* 运动状态标志位 */
   	uint8_t    state[2];     						 //VisionState_e  0x01->Comm_Successed
	uint8_t    data_tail;  	 						 //尾帧
} Comm_VisionInfo_t;


typedef struct
{
		uint8_t 	 data_head;
		uint8_t 	 vx[4];
		uint8_t 	 vy[4];
		uint8_t 	 vw[4];
		uint8_t    data_tail; 
} Comm_NavInfo_t;

#pragma pack(pop)


/* 宏定义 --------------------------------------------------------------------*/
#define COMPUTER_PROTOCOL_HEADER_SOF     0x55
#define COMPUTER_DATA_FIFO_SIZE          (256u)
/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void ComputerProtocol_ParseHandler(uint16_t cmd_id, uint8_t* data, uint16_t len);
Comm_VisionInfo_t* VisionInfo_Pointer(void);
Comm_NavInfo_t* NavInfo_Pointer(void);

Comm_RobotInfo_t* RobotInfo_Pointer(void);
Comm_DecisionInfo_t* DecisionInfo_Pointer(void);

int TwoBytesToInt (uint8_t byte[2]);
float FourBytesToFloat (uint8_t byte[4]);
#endif  // VISION_PROTOCOL_H

