// 实际BMI088与Imu源码，HAL寄存器总线替身；故障也返回合理字节，验证必须检查状态。
#include <cmath>
#include <cstdio>
#include <cstring>
#include "imu.hpp"
uint32_t now_ms=0, sensor_time=100;
bool spi_failure=false, acc_ready=true, gyro_ready=true;
GPIO_TypeDef acc_port,gyro_port; SPI_HandleTypeDef spi;
uint8_t regs[2][128]{}; int selected=0; uint8_t address=0; bool header=false;
int transfer_count=0, fail_on_transfer=-1;
namespace hello_world::tick {
uint32_t GetTickMs() { return now_ms; }
uint64_t GetTickUs() { return uint64_t(now_ms)*1000; }
uint32_t GetTickS() { return now_ms/1000; }
void DelayUs(uint32_t) {}
}
void HAL_Delay(uint32_t n) { now_ms+=n; }
void HAL_GPIO_WritePin(GPIO_TypeDef* p,uint16_t,GPIO_PinState state) {
  if(state==GPIO_PIN_RESET) { selected=p==&acc_port?1:2; header=false; }
  else selected=0;
}
uint8_t Read(int chip,uint8_t reg) {
  if(chip==0 && reg>=0x18 && reg<=0x1A) return uint8_t(sensor_time>>(8*(reg-0x18)));
  if(chip==0 && reg==0x03) return acc_ready?0x80:0;
  if(chip==1 && reg==0x0A) return gyro_ready?0x80:0;
  return regs[chip][reg&127];
}
HAL_StatusTypeDef Result() { ++transfer_count; return spi_failure || transfer_count==fail_on_transfer ? HAL_TIMEOUT:HAL_OK; }
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef*,uint8_t* tx,uint16_t n,uint32_t) {
  if(selected && n==2) regs[selected-1][tx[0]&127]=tx[1];
  else if(n==1) { address=tx[0]&127; header=true; }
  return Result();
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef*,uint8_t* tx,uint8_t* rx,uint16_t n,uint32_t) {
  std::memset(rx,0,n);
  if(selected) {
    const int chip=selected-1;
    if(!header) {
      address=tx[0]&127;
      if((chip==0 && n==3)||(chip==1 && n==2)) rx[n-1]=Read(chip,address);
      header=true;
    } else for(int i=0;i<n;++i) rx[i]=Read(chip,uint8_t(address+i));
  }
  return Result();
}
int checks=0,failures=0;
#define CHECK(x) do { ++checks; if(!(x)){ ++failures;std::printf("FAIL %d: %s\n",__LINE__,#x); } } while(0)
void Tick(hello_world::imu::Imu& imu,bool advance=true) {
  ++now_ms; if(advance) sensor_time=(sensor_time+26)&0xFFFFFF; imu.update();
}
void Init(hello_world::imu::Imu& imu) {
  spi_failure=false;acc_ready=gyro_ready=true;fail_on_transfer=-1;
  regs[0][0]=0x1E;regs[1][0]=0x0F;regs[0][0x16]=0xAB;regs[0][0x17]=0x2A;
  hello_world::imu::BMI088HWConfig hw;hw.hspi=&spi;hw.acc_cs_port=&acc_port;hw.gyro_cs_port=&gyro_port;
  CHECK(imu.init(hw));
  for(int i=0;i<2000;++i) Tick(imu);
  CHECK(imu.isCalibrated()); CHECK(imu.isSampleValid());
}
int main() {
  hello_world::imu::Imu imu; Init(imu);
  const int before=bmi_ahrs_updates; spi_failure=true;Tick(imu);
  CHECK(!imu.isSampleValid());CHECK(bmi_ahrs_updates==before);
  spi_failure=false;Tick(imu);CHECK(imu.isSampleValid());
  // 一次传输失败也不得让其它成功字段掩盖。
  fail_on_transfer=transfer_count+2;Tick(imu);CHECK(!imu.isSampleValid());fail_on_transfer=-1;
  Tick(imu);CHECK(imu.isSampleValid());
  // 数值一直相同但硬件时间推进，是正常静止；不按数值相等判冻结。
  for(int i=0;i<30;++i) { Tick(imu); }
  CHECK(imu.isSampleValid());
  for(int i=0;i<21;++i) { Tick(imu,false); }
  CHECK(!imu.isSampleValid());
  Tick(imu);CHECK(imu.isSampleValid());
  gyro_ready=false;for(int i=0;i<21;++i) Tick(imu);CHECK(!imu.isSampleValid());
  gyro_ready=true;Tick(imu);CHECK(imu.isSampleValid());
  acc_ready=false;for(int i=0;i<21;++i) Tick(imu);CHECK(!imu.isSampleValid());
  acc_ready=true;Tick(imu);CHECK(imu.isSampleValid());
  now_ms+=21;CHECK(!imu.isSampleValid()); // 跳过update也必须到期，涵盖原冻结故障注入。
  Init(imu); sensor_time=0xFFFFF0;Tick(imu);Tick(imu);CHECK(imu.isSampleValid());
  sensor_time=(sensor_time-1000)&0xFFFFFF;Tick(imu,false);CHECK(!imu.isSampleValid());CHECK(!imu.isCalibrated());
  std::printf("BMI transport: %d checks, %d failures\n",checks,failures);
  return failures?1:0;
}
