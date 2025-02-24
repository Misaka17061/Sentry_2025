/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-08-27 15:09:11
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-09-05 15:44:44
 * @FilePath: \MDK-ARMd:\RM_work\Hero_GroupWork\Hero2025_V1\Components\drvices\Super_Capacitor\super_capacitor.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef SUPER_CAPACITOR_H
#define SUPER_CAPACITOR_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "stdint.h"
#include "bsp_init.h"
#include "referee_system.h"
//#include "Motor/motor.h"
#include "chassis/chassis_app.h"
/* 类型定义 ------------------------------------------------------------------*/
typedef struct {
    float in_v;       
    float cap_v;       
    float in_c;        
    uint8_t cap_percent;  
    uint8_t in_power;    
} Cap_power;
/* 宏定义 --------------------------------------------------------------------*/



/* 扩展变量 ------------------------------------------------------------------*/
extern Cap_power cap_info;
/* 函数声明 ------------------------------------------------------------------*/
void CAP_PowerParser(Cap_power *received_data, uint8_t *can_receive_data, uint16_t len);
Cap_power* CAP_GetDataPointer(void); //传出一个*P指针访问值。
void SuperCap_SendMessage(CAN_Object_t* obj);
#endif 
