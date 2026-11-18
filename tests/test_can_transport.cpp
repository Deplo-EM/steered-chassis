// 实际HW_can源码，模拟邮箱寄存器与完成状态，不能把HAL接受当成发送完成。
#include <cstdio>
#include <cstring>
#include "HW_can.hpp"
#include "ins_all.hpp"
CAN_TypeDef reg1,reg2;
CAN_HandleTypeDef hcan1{&reg1},hcan2{&reg2};
bool abort_ok=true, abort_deferred=false, complete_before_add=false;
uint32_t now=0; int aborts=0,checks=0,failures=0;
namespace hello_world::tick {
uint32_t GetTickMs(){return now;}
uint64_t GetTickUs(){return uint64_t(now)*1000;}
uint32_t GetTickS(){return now/1000;}
void DelayUs(uint32_t){}
}
uint32_t filter_split[2]={};
int HAL_CAN_ConfigFilter(CAN_HandleTypeDef* h,CAN_FilterTypeDef* f){filter_split[h==&hcan1?0:1]=f->SlaveStartFilterBank;return HAL_OK;}
int HAL_CAN_AddTxMessage(CAN_HandleTypeDef* h,CAN_TxHeaderTypeDef*,uint8_t*,uint32_t* mailbox){
  if(before_tx_request){auto hook=before_tx_request;before_tx_request=nullptr;hook();}
  if(complete_before_add) { // 硬件在预检后、HAL选邮箱前完成：返回的邮箱仍被软件追踪。
    complete_before_add=false;
    h->Instance->TSR|=CAN_TSR_TME0|9u;
  }
  for(int i=0;i<3;++i) if(h->Instance->TSR&(CAN_TSR_TME0<<i)) {
    h->Instance->TSR=1u<<(8*i); // HAL真实行为：入邮箱前W1C旧完成位
    h->Instance->TSR&=~(CAN_TSR_TME0<<i);*mailbox=1u<<i;return HAL_OK;
  }
  return HAL_ERROR;
}
int HAL_CAN_AbortTxRequest(CAN_HandleTypeDef* h,uint32_t mask){
  ++aborts;
  if(!abort_ok) return HAL_ERROR;
  if(abort_deferred) return HAL_OK;
  for(int i=0;i<3;++i) if(mask&(1u<<i)) h->Instance->TSR|=CAN_TSR_TME0<<i;
  return HAL_OK;
}
bool rx_ready=false;uint32_t rx_id=0;uint8_t rx_payload[8]={};
int HAL_CAN_GetRxMessage(CAN_HandleTypeDef*,uint32_t,CAN_RxHeaderTypeDef* header,uint8_t* data){
  if(!rx_ready)return HAL_ERROR;
  rx_ready=false;header->StdId=rx_id;header->IDE=CAN_ID_STD;header->RTR=CAN_RTR_DATA;header->DLC=8;
  std::memcpy(data,rx_payload,8);return HAL_OK;
}
int HAL_CAN_ActivateNotification(CAN_HandleTypeDef*,uint32_t){return HAL_OK;}
void Error_Handler(){}
robot::Robot* GetRobot(){return nullptr;}
robot::GimbalChassisComm relay_comm(robot::GimbalChassisComm::CodePart::Chassis,0x210,0x211);
robot::GimbalChassisComm* GetGimbalChassisComm(){return &relay_comm;}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef*);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef*);
hello_world::motor::Motor* GetMotorWheel(int){return nullptr;}
hello_world::motor::Motor* GetMotorSteer(int){return nullptr;}
hello_world::motor::Motor* GetMotorYaw(){return nullptr;}
hello_world::motor::Motor* GetMotorPitch(){return nullptr;}
void robot::Robot::notifyMotorRx(int){}
void Complete(bool success){
  for(int i=0;i<3;++i) if(!(reg1.TSR&(CAN_TSR_TME0<<i))) {
    reg1.TSR|=CAN_TSR_TME0<<i;
    reg1.TSR|=(success?3u:9u)<<(8*i);
  }
}
#define CHECK(x) do{++checks;if(!(x)){++failures;std::printf("FAIL %d: %s\n",__LINE__,#x);}}while(0)
int main(){
  CanFilter_Init(&hcan1); CanFilter_Init(&hcan2);
  CHECK(filter_split[0]==14u); CHECK(filter_split[1]==14u);
  uint8_t f[8]={};
  CHECK(CAN_Send_Msg(&hcan1,f,0x200,8));Complete(true);
  ++now;CHECK(CAN_Send_Msg(&hcan1,f,0x200,8));
  now+=3;CHECK(!CAN_Send_Msg(&hcan1,f,0x200,8));CHECK(aborts>0);
  Complete(true);CHECK(CAN_Send_Msg(&hcan1,f,0x200,8));Complete(false);
  const int count=aborts;CHECK(!CAN_Send_Msg(&hcan1,f,0x200,8));CHECK(aborts>count);
  Complete(true);now=0xFFFFFFFEu;CHECK(CAN_Send_Msg(&hcan1,f,0x200,8));
  now=1;CHECK(!CAN_Send_Msg(&hcan1,f,0x200,8));
  // 三邮箱独立追踪，清RQCP不能抹掉其他邮箱的状态。
  CHECK(CAN_AbortPendingTx(&hcan1));
  CHECK(CAN_Send_Msg(&hcan1,f,0x200,8));
  CHECK(CAN_Send_Msg(&hcan1,f,0x201,8));
  CHECK(CAN_Send_Msg(&hcan1,f,0x202,8));
  CHECK(!CAN_Send_Msg(&hcan1,f,0x203,8));
  CHECK(CAN_TrySend(&hcan1,f,0x210,8) == robot::CanTxSubmitResult::kBusy);
  reg1.TSR|=CAN_TSR_TME0|3u;
  CHECK(CAN_TxHealthy(&hcan1)); CHECK((reg1.TSR&3u)==0u);
  reg1.TSR|=(CAN_TSR_TME0<<1)|(9u<<8);
  CHECK(!CAN_TxHealthy(&hcan1));
  CHECK(CAN_Send_Msg(&hcan2,f,0x210,8)); // 总线2不能复用总线1的监督状态
  CHECK(CAN_TxHealthy(&hcan1)); now+=2;
  CHECK(!CAN_TxHealthy(&hcan2));
  CHECK(!CAN_Send_Msg(nullptr,f,0x200,8));
  CHECK(!CAN_Send_Msg(&hcan1,nullptr,0x200,8));
  CHECK(!CAN_Send_Msg(&hcan1,f,0x800,8));
  CHECK(!CAN_Send_Msg(&hcan1,f,0x200,9));
  reg1.ESR=CAN_ESR_BOFF;
  CHECK(CAN_TrySend(&hcan1,f,0x210,8) == robot::CanTxSubmitResult::kFault);
  CHECK(!CAN_TxHealthy(&hcan1)); reg1.ESR=0;
  // HAL接受撤销请求不等于邮箱已空；必须保留追踪并等待确认，不能遗忘旧帧。
  CHECK(CAN_Send_Msg(&hcan1,f,0x200,8));
  abort_deferred=true;
  CHECK(!CAN_AbortPendingTx(&hcan1));
  now+=2; CHECK(!CAN_TxHealthy(&hcan1));
  abort_deferred=false; abort_ok=false;
  CHECK(!CAN_AbortPendingTx(&hcan1));
  abort_ok=true; CHECK(CAN_AbortPendingTx(&hcan1));
  CHECK(CAN_TxHealthy(&hcan1));
  CHECK(CAN_Send_Msg(&hcan1,f,0x200,8));
  before_tx_request=[](){reg1.TSR|=CAN_TSR_TME0|3u;};
  CHECK(CAN_Send_Msg(&hcan1,f,0x201,8)); // 旧邮箱在选槽期间成功完成，不能误触发故障
  CHECK(CAN_TxHealthy(&hcan1));
  Complete(true);CHECK(CAN_TxHealthy(&hcan1));
  CHECK(CAN_Send_Msg(&hcan1,f,0x200,8));
  before_tx_request=[](){reg1.TSR|=CAN_TSR_TME0|9u;};
  CHECK(CAN_Send_Msg(&hcan1,f,0x201,8));
  CHECK(!CAN_TxHealthy(&hcan1)); // 旧邮箱失败也不能被新请求抹掉
  uint8_t payload[8]={0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};
  CHECK(CAN_Send_Msg(&hcan1,payload,0x321,8));
  CHECK(uint32_t(reg1.sTxMailBox[0].TIR)==0x64200001u);
  CHECK(reg1.sTxMailBox[0].TDTR==8u);
  CHECK(reg1.sTxMailBox[0].TDLR==0x44332211u);
  CHECK(reg1.sTxMailBox[0].TDHR==0x88776655u);
  Complete(true);
  CHECK(CAN_TxHealthy(&hcan1));
  // 调用方只授权读取 DLC 个字节，缓冲区尾部不能带进邮箱。
  CHECK(CAN_Send_Msg(&hcan1,payload,0x321,3));
  CHECK(reg1.sTxMailBox[0].TDTR==3u);
  CHECK(reg1.sTxMailBox[0].TDLR==0x00332211u && reg1.sTxMailBox[0].TDHR==0u);
  CHECK(CAN_AbortPendingTx(&hcan1));hcan1.State=0;
  CHECK(!CAN_Send_Msg(&hcan1,payload,0x321,8));hcan1.State=HAL_CAN_STATE_LISTENING;

  // 超时后的撤销并非同步完成，即使有其他空邮箱也不得提交新指令。
  CHECK(CAN_Send_Msg(&hcan1,f,0x1FF,8));
  now+=2; abort_deferred=true;
  CHECK(!CAN_TxHealthy(&hcan1));
  CHECK((reg1.TSR & CAN_TSR_TME0)==0);
  CHECK(!CAN_TxHealthy(&hcan1));
  CHECK(!CAN_Send_Msg(&hcan1,f,0x1FF,8));
  abort_deferred=false;
  CHECK(CAN_AbortPendingTx(&hcan1));
  CHECK(CAN_TrySend(&hcan1,f,0x210,8) == robot::CanTxSubmitResult::kQueued);
  Complete(true);
  CHECK(CAN_TxHealthy(&hcan1));
  robot::CanTxReceipt receipt;
  CHECK(CAN_SendTracked(&hcan1,f,0x1FF,8,receipt));
  CHECK(receipt.status == robot::CanTxStatus::kPending);
  Complete(true);
  CHECK(CAN_TxHealthy(&hcan1));
  CHECK(receipt.status == robot::CanTxStatus::kSent);
  CHECK(CAN_SendTracked(&hcan1,f,0x1FF,8,receipt));
  now+=2; abort_deferred=true;
  CHECK(!CAN_TxHealthy(&hcan1));
  CHECK(receipt.status == robot::CanTxStatus::kUnconfirmed);
  CHECK(!CAN_SendTracked(&hcan1,f,0x1FF,8,receipt));
  abort_deferred=false;
  CHECK(CAN_AbortPendingTx(&hcan1));
#ifdef TEST_CHASSIS
  robot::DbusRelay source;
  uint8_t raw[18]={0,4,0x20,0,1,0x68,0,0,0,0,0,0,0,0,0,0,0,4};
  CHECK(source.publish(raw,now));
  for(int part=0;part<3;++part) {
    CHECK(source.peekTx(now,rx_id,rx_payload));rx_ready=true;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);source.txAccepted();
  }
  CHECK(!relay_comm.remote_relay().online(now)); // 错误总线不能授权遥控。
  CHECK(source.publish(raw,now+1));
  ++now;
  for(int part=0;part<3;++part) {
    CHECK(source.peekTx(now,rx_id,rx_payload));rx_ready=true;
    HAL_CAN_RxFifo1MsgPendingCallback(&hcan2);source.txAccepted();
  }
  CHECK(relay_comm.remote_relay().online(now));
  CHECK(std::memcmp(relay_comm.remote_relay().frame(),raw,18)==0);
#endif
  std::printf("CAN transport: %d checks, %d failures\n",checks,failures);
  return failures?1:0;
}
