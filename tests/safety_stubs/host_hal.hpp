#pragma once
#include <cstdint>
inline uint32_t host_irq_mask = 0;
inline uint32_t __get_PRIMASK() { return host_irq_mask; }
inline void __disable_irq() { host_irq_mask=1; }
inline void __enable_irq() { host_irq_mask=0; }
inline void __set_PRIMASK(uint32_t value) { host_irq_mask=value; }
inline void __DMB() {}
struct CAN_HandleTypeDef {};
struct DMA_HandleTypeDef {};
inline DMA_HandleTypeDef fake_uart_dma;
struct UART_HandleTypeDef { DMA_HandleTypeDef* hdmarx=&fake_uart_dma; };
struct IWDG_HandleTypeDef {};
extern CAN_HandleTypeDef hcan1, hcan2;
extern UART_HandleTypeDef huart3;
extern IWDG_HandleTypeDef hiwdg;
constexpr int CAN_IT_RX_FIFO0_MSG_PENDING = 1;
constexpr int CAN_IT_RX_FIFO1_MSG_PENDING = 2;
constexpr uint32_t HAL_CAN_ERROR_BOF = 4;
constexpr int HAL_OK = 0;
constexpr int HAL_ERROR=1, DMA_IT_HT=1,HAL_UART_RXEVENT_IDLE=0,HAL_UART_RXEVENT_HT=1;
inline int fake_rx_event=HAL_UART_RXEVENT_IDLE;
inline int HAL_UARTEx_GetRxEventType(UART_HandleTypeDef*) {return fake_rx_event;}
inline void __HAL_DMA_DISABLE_IT(DMA_HandleTypeDef*,int) {}
int HAL_CAN_Start(CAN_HandleTypeDef*);
int HAL_CAN_ActivateNotification(CAN_HandleTypeDef*, int);
uint32_t HAL_CAN_GetError(const CAN_HandleTypeDef*);
int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef*, uint8_t*, uint16_t);
int HAL_IWDG_Refresh(IWDG_HandleTypeDef*);
