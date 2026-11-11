#pragma once
#include <cstdint>
struct CAN_TypeDef { bool single_shot=false; bool pending=false; };
struct CAN_InitTypeDef {
  uint32_t Prescaler,Mode,SyncJumpWidth,TimeSeg1,TimeSeg2,TimeTriggeredMode,
    AutoBusOff,AutoWakeUp,AutoRetransmission,ReceiveFifoLocked,TransmitFifoPriority;
};
struct CAN_HandleTypeDef { CAN_TypeDef* Instance; CAN_InitTypeDef Init; };
inline CAN_TypeDef test_can1, test_can2;
#define CAN1 (&test_can1)
#define CAN2 (&test_can2)
constexpr int HAL_OK=0,ENABLE=1,DISABLE=0;
constexpr int CAN_MODE_NORMAL=0,CAN_SJW_1TQ=1,CAN_BS1_9TQ=9,CAN_BS2_4TQ=4;
struct GPIO_InitTypeDef { uint32_t Pin,Mode,Pull,Speed,Alternate; };
constexpr int GPIO_PIN_0=1,GPIO_PIN_1=2,GPIO_PIN_5=32,GPIO_PIN_6=64;
constexpr int GPIO_MODE_AF_PP=2,GPIO_NOPULL=0,GPIO_SPEED_FREQ_VERY_HIGH=3;
constexpr int GPIO_AF9_CAN1=9,GPIO_AF9_CAN2=9,GPIOB=1,GPIOD=2;
constexpr int CAN1_TX_IRQn=0,CAN1_RX0_IRQn=1,CAN1_SCE_IRQn=2,CAN2_TX_IRQn=3,CAN2_RX1_IRQn=4,CAN2_SCE_IRQn=5;
inline int HAL_CAN_Init(CAN_HandleTypeDef* h) {
  h->Instance->single_shot=(h->Init.AutoRetransmission==DISABLE);return HAL_OK;
}
inline void HAL_GPIO_Init(int,GPIO_InitTypeDef*) {}
inline void HAL_GPIO_DeInit(int,int) {}
inline void HAL_NVIC_SetPriority(int,int,int) {}
inline void HAL_NVIC_EnableIRQ(int) {}
inline void HAL_NVIC_DisableIRQ(int) {}
inline void Error_Handler() {}
#define __HAL_RCC_CAN1_CLK_ENABLE() ((void)0)
#define __HAL_RCC_CAN2_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
#define __HAL_RCC_CAN1_CLK_DISABLE() ((void)0)
#define __HAL_RCC_CAN2_CLK_DISABLE() ((void)0)
extern CAN_HandleTypeDef hcan1,hcan2;
void MX_CAN1_Init();
void MX_CAN2_Init();
