#pragma once
#include <cstdint>
inline void __disable_irq() {}
inline void __enable_irq() {}
struct CAN_HandleTypeDef {};
struct UART_HandleTypeDef {};
struct IWDG_HandleTypeDef {};
extern CAN_HandleTypeDef hcan1, hcan2;
extern UART_HandleTypeDef huart3;
extern IWDG_HandleTypeDef hiwdg;
constexpr int CAN_IT_RX_FIFO0_MSG_PENDING = 1;
constexpr int CAN_IT_RX_FIFO1_MSG_PENDING = 2;
constexpr uint32_t HAL_CAN_ERROR_BOF = 4;
constexpr int HAL_OK = 0;
int HAL_CAN_Start(CAN_HandleTypeDef*);
int HAL_CAN_ActivateNotification(CAN_HandleTypeDef*, int);
uint32_t HAL_CAN_GetError(const CAN_HandleTypeDef*);
int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef*, uint8_t*, uint16_t);
int HAL_IWDG_Refresh(IWDG_HandleTypeDef*);
