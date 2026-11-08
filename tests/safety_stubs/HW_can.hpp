#pragma once
#include "host_hal.hpp"
void CanFilter_Init(CAN_HandleTypeDef*);
bool CAN_Send_Msg(CAN_HandleTypeDef*, uint8_t*, uint32_t, uint8_t);

bool CAN_Is_BusOff(const CAN_HandleTypeDef*);
