#pragma once
#include <cstdint>
struct TestTsr {
  uint32_t value=0x1C000000u;
  operator uint32_t() const { return value; }
  TestTsr& operator=(uint32_t mask) {
    for(int i=0;i<3;++i) if(mask&(1u<<(8*i))) value&=~(0xFu<<(8*i));
    return *this;
  }
  TestTsr& operator|=(uint32_t mask) { value|=mask;return *this; }
  TestTsr& operator&=(uint32_t mask) { value&=mask;return *this; }
};
inline void (*before_tx_request)()=nullptr;
struct TestTir {
  uint32_t value=0; TestTsr* tsr=nullptr; int index=0;
  operator uint32_t() const {return value;}
  TestTir& operator=(uint32_t v) {
    if(v&1u) {
      if(before_tx_request){auto hook=before_tx_request;before_tx_request=nullptr;hook();}
      tsr->value&=~(1u<<(26+index));
    }
    value=v;return *this;
  }
  TestTir& operator|=(uint32_t v) {
    if(v&1u) {
      if(before_tx_request){auto hook=before_tx_request;before_tx_request=nullptr;hook();}
      tsr->value&=~(1u<<(26+index));
    }
    value|=v;return *this;
  }
};
struct TestMailbox {TestTir TIR;uint32_t TDTR=0,TDLR=0,TDHR=0;};
struct CAN_TypeDef {
  uint32_t ESR=0; TestTsr TSR; TestMailbox sTxMailBox[3];
  CAN_TypeDef(){for(int i=0;i<3;++i){sTxMailBox[i].TIR.tsr=&TSR;sTxMailBox[i].TIR.index=i;}}
};
constexpr uint32_t HAL_CAN_STATE_READY=1,HAL_CAN_STATE_LISTENING=2;
struct CAN_HandleTypeDef { CAN_TypeDef* Instance=nullptr;uint32_t State=HAL_CAN_STATE_LISTENING; };
struct CAN_FilterTypeDef { uint32_t FilterMode,FilterScale,FilterIdHigh,FilterIdLow,FilterMaskIdHigh,FilterMaskIdLow,FilterActivation,FilterFIFOAssignment,FilterBank,SlaveStartFilterBank; };
struct CAN_TxHeaderTypeDef { uint32_t StdId,ExtId,IDE,RTR,DLC,TransmitGlobalTime; };
struct CAN_RxHeaderTypeDef { uint32_t StdId,ExtId,IDE,RTR,DLC; };
constexpr int HAL_OK=0, HAL_ERROR=1,ENABLE=1;
constexpr uint32_t CAN_FILTERMODE_IDMASK=0,CAN_FILTERSCALE_32BIT=0,CAN_FILTER_FIFO0=0,CAN_FILTER_FIFO1=1;
constexpr uint32_t CAN_ID_STD=0,CAN_RTR_DATA=0,CAN_RX_FIFO0=0,CAN_RX_FIFO1=1;
constexpr uint32_t CAN_IT_RX_FIFO0_MSG_PENDING=1,CAN_IT_RX_FIFO1_MSG_PENDING=2;
constexpr uint32_t CAN_ESR_BOFF=4,CAN_TSR_TME0=1u<<26,CAN_TSR_TXOK0=2;
constexpr uint32_t CAN_TX_MAILBOX0=1,CAN_TX_MAILBOX1=2,CAN_TX_MAILBOX2=4;
constexpr uint32_t CAN_FLAG_RQCP0=0,CAN_FLAG_RQCP1=1,CAN_FLAG_RQCP2=2;
inline void __disable_irq() {}
inline void __enable_irq() {}
inline uint32_t __get_PRIMASK() {return 0;}
inline void __set_PRIMASK(uint32_t) {}
inline void __DMB() {}
constexpr uint32_t CAN_TI0R_STID_Pos=21,CAN_TI0R_TXRQ=1;
inline void ClearCanFlag(CAN_HandleTypeDef* h,uint32_t i) { h->Instance->TSR&=~(0xFu<<(8*i)); }
#define __HAL_CAN_CLEAR_FLAG(h,f) ClearCanFlag(h,f)
int HAL_CAN_ConfigFilter(CAN_HandleTypeDef*,CAN_FilterTypeDef*);
int HAL_CAN_AddTxMessage(CAN_HandleTypeDef*,CAN_TxHeaderTypeDef*,uint8_t*,uint32_t*);
int HAL_CAN_AbortTxRequest(CAN_HandleTypeDef*,uint32_t);
int HAL_CAN_GetRxMessage(CAN_HandleTypeDef*,uint32_t,CAN_RxHeaderTypeDef*,uint8_t*);
int HAL_CAN_ActivateNotification(CAN_HandleTypeDef*,uint32_t);
void Error_Handler();
extern CAN_HandleTypeDef hcan1,hcan2;
