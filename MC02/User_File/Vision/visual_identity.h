/**
 ******************************************************************************
 * @file    visual_identity.h
 * @brief   视觉通信接口与数据定义
 ******************************************************************************
 */

#ifndef VISUAL_IDENTITY_H
#define VISUAL_IDENTITY_H

#include "main.h"
#include <stdint.h>

/* 视觉接收数据 */
typedef struct
{
    int16_t r_block_x;
    int16_t r_block_y;

    int16_t g_block_x;
    int16_t g_block_y;

    int16_t b_block_x;
    int16_t b_block_y;

    int16_t rgb_circle_x;
    int16_t rgb_circle_y;
    int16_t rgb_circle_z;   /* 左环减右环 */

    int16_t tray_center_x;
    int16_t tray_center_y;

    uint8_t block_grab;
    uint8_t block_order;
} VISUAL_DATA_t;

/* 全局视觉数据 */
extern VISUAL_DATA_t VIS_RX;
extern uint8_t color_task[6];

/* 视觉数据初始化 */
void Visual_Data_Init(void);

/* 向视觉模块发送任务 */
void visual_idle(void);
void identify_QR_code(void);
void identify_material_tray(void);
void identify_materials(uint8_t color);
void request_grab_materials(uint8_t color);
void identify_color_rings(void);
void stacking_positioning(void);
void identify_material_tray_final(void);
void request_to_place_materials(void);

/* 接收数据解包 */
void OPENMV_Data_Unpack(uint8_t openmv_data[16]);
void NANO_Data_Unpack(uint8_t nano_data[16]);

#endif /* VISUAL_IDENTITY_H */