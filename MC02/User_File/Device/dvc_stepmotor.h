#ifndef DVC_STEPMOTOR_H
#define DVC_STEPMOTOR_H

#include "drv_can.h"
#include <stdbool.h>


void Emm_V5_Trig_Encoder_Cal(uint8_t addr);

void Emm_V5_En_Control(uint8_t addr, bool state, bool snF);

void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);

void Emm_V5_Stop_Now(uint8_t addr, bool snF);

void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF, bool snF);

void Emm_V5_Angle_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint16_t angle, uint8_t raF, bool snF);




#endif // DVC_STEPMOTOR_H   