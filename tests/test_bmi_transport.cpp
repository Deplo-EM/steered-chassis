// 实际BMI088与Imu源码，HAL寄存器总线替身；故障也返回合理字节，验证必须检查状态。
#include <cmath>
#include <array>
#include <cstdio>
#include <cstring>
#include <deque>
#include "imu.hpp"
uint32_t now_ms=0, sensor_time=100;
bool spi_failure=false, acc_ready=true, gyro_ready=true;
bool fail_temperature=false;
bool gyro_pulse_visible=true, fifo_overrun=false;
int gyro_frames_per_tick=1;
std::deque<std::array<int16_t,3>> gyro_fifo;
GPIO_TypeDef acc_port,gyro_port; SPI_HandleTypeDef spi;
uint8_t regs[2][128]{}; int selected=0; uint8_t address=0; bool header=false;
int transfer_count=0, fail_on_transfer=-1;
uint32_t gyro_reset_tick=0;
bool gyro_reset_seen=false, gyro_config_before_reset=false;
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
  if(chip==1 && reg==0x0A) return gyro_ready && gyro_pulse_visible ? 0x80:0;
  if(chip==1 && reg==0x0E) return uint8_t(gyro_fifo.size()) | (fifo_overrun ? 0x80:0);
  return regs[chip][reg&127];
}
HAL_StatusTypeDef Result() { ++transfer_count; return spi_failure || transfer_count==fail_on_transfer ? HAL_TIMEOUT:HAL_OK; }
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef*,uint8_t* tx,uint16_t n,uint32_t) {
  if(selected && n==2) {
    const uint8_t reg=tx[0]&127;
    if(selected==2 && reg==0x14 && tx[1]==0xB6) {
      gyro_reset_tick=now_ms;gyro_reset_seen=true;
    } else if(selected==2 && gyro_reset_seen && now_ms-gyro_reset_tick<30u) {
      gyro_config_before_reset=true;
    }
    regs[selected-1][reg]=tx[1];
    if(selected==2 && reg==0x3E) { gyro_fifo.clear();fifo_overrun=false; }
  }
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
    } else if(chip==1 && address==0x3F) {
      for(uint16_t offset=0;offset+6<=n && !gyro_fifo.empty();offset+=6) {
        const auto frame=gyro_fifo.front();gyro_fifo.pop_front();
        for(int axis=0;axis<3;++axis) {
          const uint16_t raw=uint16_t(frame[axis]);
          rx[offset+2*axis]=uint8_t(raw);rx[offset+2*axis+1]=uint8_t(raw>>8);
        }
      }
    } else for(int i=0;i<n;++i) rx[i]=Read(chip,uint8_t(address+i));
  }
  const auto result=Result();
  if(fail_temperature && selected==1 && address==0x22) return HAL_TIMEOUT;
  return result;
}
int checks=0,failures=0;
#define CHECK(x) do { ++checks; if(!(x)){ ++failures;std::printf("FAIL %d: %s\n",__LINE__,#x); } } while(0)
void Tick(hello_world::imu::Imu& imu,bool advance=true) {
  ++now_ms; if(advance) sensor_time=(sensor_time+26)&0xFFFFFF;
  if(gyro_ready && (regs[1][0x3E]&0xC3)==0x80) {
    for(int i=0;i<gyro_frames_per_tick;++i) {
      if(gyro_fifo.size()==99) { fifo_overrun=true;gyro_fifo.pop_front(); }
      gyro_fifo.push_back({0,0,0});
    }
  }
  imu.update();
}
void Init(hello_world::imu::Imu& imu,bool pulse_visible=true,
          const hello_world::imu::BMI088Config& config={}) {
  spi_failure=false;acc_ready=gyro_ready=true;fail_on_transfer=-1;
  gyro_pulse_visible=pulse_visible;gyro_frames_per_tick=1;gyro_fifo.clear();fifo_overrun=false;
  gyro_reset_seen=false;gyro_config_before_reset=false;
  regs[0][0]=0x1E;regs[1][0]=0x0F;regs[0][0x16]=0xAB;regs[0][0x17]=0x2A;
  hello_world::imu::BMI088HWConfig hw;hw.hspi=&spi;hw.acc_cs_port=&acc_port;hw.gyro_cs_port=&gyro_port;
  CHECK(imu.init(hw, hello_world::imu::kImuDefaultRotMat, config));
  CHECK(gyro_reset_seen && !gyro_config_before_reset);
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
  // 正常陀螺仪仍在1kHz出样，但1ms轮询相位始终落在280–400us脉冲之外。
  Init(imu,false);
  // 温度读取失败时不得先丢掉已经入队的陀螺样本；下一拍可补回积分。
  const int before_temp_failure=bmi_ahrs_updates;
  fail_temperature=true;Tick(imu);fail_temperature=false;
  CHECK(!imu.isSampleValid());CHECK(gyro_fifo.size()==1);
  Tick(imu);CHECK(bmi_ahrs_updates==before_temp_failure+2);
  const int before_empty=bmi_ahrs_updates;
  gyro_ready=false;for(int i=0;i<21;++i) Tick(imu);
  CHECK(!imu.isSampleValid());CHECK(bmi_ahrs_updates==before_empty);
  gyro_ready=true;Tick(imu);CHECK(imu.isSampleValid());
  const int before_batch=bmi_ahrs_updates;
  gyro_frames_per_tick=2;Tick(imu);CHECK(bmi_ahrs_updates==before_batch+2);
  gyro_frames_per_tick=5;Tick(imu);CHECK(!imu.isSampleValid());CHECK(gyro_fifo.empty());
  gyro_frames_per_tick=1;Tick(imu);CHECK(imu.isSampleValid());
  fifo_overrun=true;Tick(imu);CHECK(!imu.isSampleValid());CHECK(!fifo_overrun);
  // 每次消费的是硬件帧，姿态积分时间基准必须跟硬件ODR走。
  hello_world::imu::BMI088Config high_rate;
  high_rate.gyro_odr_fbw=hello_world::imu::kBMI088GyroOdrFbw2000_230;
  Init(imu,false,high_rate);CHECK(bmi_ahrs_frequency==2000.0f);
  imu.restartCalibration();CHECK(bmi_ahrs_frequency==2000.0f);
  std::printf("BMI transport: %d checks, %d failures\n",checks,failures);
  return failures?1:0;
}
