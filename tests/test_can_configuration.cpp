// 编译真实can.c；HAL边界保存初始化参数，再按RM0090描述模拟正常仲裁失败。
#include <cstdio>
#include <initializer_list>
#include "can.h"
int main() {
  MX_CAN1_Init();MX_CAN2_Init();
  int failures=0;
  for(auto* h:{&hcan1,&hcan2}) {
    h->Instance->pending=true;
    if(h->Instance->single_shot) h->Instance->pending=false;
    if(!h->Instance->pending) {
      ++failures;std::puts("FAIL: normal arbitration loss discarded the pending command");
    }
  }
  std::printf("CAN configuration: 2 checks, %d failures\n",failures);
  return failures?1:0;
}
