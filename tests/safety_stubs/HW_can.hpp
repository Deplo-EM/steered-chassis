#pragma once
#include "host_hal.hpp"
#include "can_tx_receipt.hpp"
void CanFilter_Init(CAN_HandleTypeDef*);
bool CAN_Send_Msg(CAN_HandleTypeDef*, uint8_t*, uint32_t, uint8_t);
robot::CanTxSubmitResult CAN_TrySend(CAN_HandleTypeDef*, const uint8_t*, uint32_t, uint8_t);

bool CAN_Is_BusOff(const CAN_HandleTypeDef*);

bool CAN_TxHealthy(CAN_HandleTypeDef*);
bool CAN_AbortPendingTx(CAN_HandleTypeDef*);
bool CAN_SendTracked(CAN_HandleTypeDef*, const uint8_t*, uint32_t, uint8_t,
                     robot::CanTxReceipt&);
