// 真实协议实例：分片损坏/丢失/重放不得变成遥控新鲜度。
#include <cstdio>
#include <cstring>
#include <type_traits>
#include "gimbal_chassis_comm.hpp"
#include "DT7.hpp"
int checks=0,failures=0;
#define CHECK(x) do{++checks;if(!(x)){++failures;std::printf("FAIL %d: %s\n",__LINE__,#x);}}while(0)
// 独立按多项式逐位计算，不使用生产CRC实现作为期望值。
uint8_t Crc(uint32_t id,const uint8_t* data) {
  uint8_t crc=0;
  for(int n=-2;n<7;++n) {
    const uint8_t b=n==-2?uint8_t(id>>8):(n==-1?uint8_t(id):data[n]);
    crc^=b;for(int k=0;k<8;++k) crc=(crc&0x80)?uint8_t((crc<<1)^7):uint8_t(crc<<1);
  }
  return crc;
}
void Packet(const uint8_t raw[18],uint8_t seq,int part,uint8_t out[8]) {
  std::memcpy(out,raw+part*6,6);out[6]=seq;out[7]=Crc(0x212+part,out);
}
template<class Comm> void Run(Comm& a,Comm& b) {
 if constexpr(requires { a.remote_relay(); }) {
  auto& tx=a.remote_relay();auto& rx=b.remote_relay();
  uint8_t raw[18]={0,4,0x20,0,1,0x68,0,0,0,0,0,0,0,0,0,0,0,4};
  uint8_t frames[3][8];uint32_t id=0;
  CHECK(!rx.online(0));CHECK(!tx.peekTx(0,id,frames[0]));
  CHECK(tx.publish(raw,0));
  for(int p=0;p<3;++p) {
    CHECK(tx.peekTx(p,id,frames[p]));CHECK(id==uint32_t(0x212+p));
    CHECK(frames[p][7]==Crc(id,frames[p]));tx.txAccepted();
  }
  CHECK(!tx.peekTx(4,id,frames[0]));
  CHECK(!rx.accept(0x214,frames[2],4));CHECK(!rx.accept(0x212,frames[0],5));
  CHECK(rx.accept(0x213,frames[1],6));CHECK(rx.online(6));
  CHECK(std::memcmp(rx.frame(),raw,18)==0);
  hello_world::remote_control::DT7 decoder;decoder.decode(rx.frame());
  CHECK(decoder.rc_r_switch()==hello_world::remote_control::kSwitchStateDown);
  CHECK(decoder.rc_lv()==0);CHECK(decoder.rc_wheel()==0);
  // 重复完整帧不续命；100ms按首次分片到达计算。
  for(int p=0;p<3;++p) CHECK(!rx.accept(0x212+p,frames[p],90));
  CHECK(rx.online(104));CHECK(!rx.online(105));
  // 离线后需两组递进完整帧重同步，单次重放不能恢复在线。
  uint8_t f[8];
  for(int p=0;p<3;++p){Packet(raw,40,p,f);CHECK(!rx.accept(0x212+p,f,110+p));}
  CHECK(!rx.online(112));
  for(int p=0;p<3;++p){Packet(raw,43,p,f);const bool ok=rx.accept(0x212+p,f,117+p);CHECK(ok==(p==2));}
  CHECK(rx.online(119));
  // CRC绑定CAN ID，换片号不能被当成合法分片。
  Packet(raw,42,0,f);CHECK(!rx.accept(0x213,f,120));
  f[0]^=1;CHECK(!rx.accept(0x212,f,121));
  // 混序号、缺片、超过20ms的拼包均不提交。
  Packet(raw,42,0,f);CHECK(!rx.accept(0x212,f,122));
  Packet(raw,43,1,f);CHECK(!rx.accept(0x213,f,123));
  Packet(raw,42,2,f);CHECK(!rx.accept(0x214,f,124));
  Packet(raw,43,2,f);CHECK(!rx.accept(0x214,f,145));
  CHECK(!rx.online(218));
  // 即使CRC正确，拨轮高位脏数据也不能穿过原DBUS校验。
  raw[16]=0;raw[17]=0xFC;
  CHECK(!tx.publish(raw,220));
  for(int p=0;p<3;++p){Packet(raw,44,p,f);CHECK(!rx.accept(0x212+p,f,220+p));}
  // 发送快照完整：新UART帧到达不能拼成两帧的混合数据。
  raw[17]=4;CHECK(tx.publish(raw,230));CHECK(tx.peekTx(230,id,f));
  const uint8_t seq=f[6];tx.txAccepted();raw[0]=1;CHECK(tx.publish(raw,231));
  CHECK(tx.peekTx(231,id,f));CHECK(f[6]==seq);tx.txAccepted();
  CHECK(tx.peekTx(232,id,f));CHECK(f[6]==seq);tx.txAccepted();
  CHECK(tx.peekTx(233,id,f));CHECK(f[0]==1);CHECK(f[6]!=seq);
  CHECK(!tx.peekTx(332,id,f)); // 过期快照不能继续发送
  // 序号255→0、系统毫秒回绕同时发生，仍按无符号时间差判断新鲜度。
  using Relay=std::remove_reference_t<decltype(rx)>;
  Relay wrap;
  for(int n=0;n<3;++n) {
    const uint8_t s=uint8_t(254+n);
    const uint32_t base=0xFFFFFFF0u+uint32_t(n*7);
    for(int p=0;p<3;++p) {
      Packet(raw,s,p,f);
      CHECK(wrap.accept(0x212+p,f,base+uint32_t(p))==(p==2));
    }
  }
  CHECK(wrap.online(98));CHECK(!wrap.online(99));
 } else { CHECK(false && "no raw DBUS relay: gimbal receiver cannot command chassis"); }
}
int main(){
 robot::GimbalChassisComm a(robot::GimbalChassisComm::CodePart::Gimbal,0x211,0x210),b(robot::GimbalChassisComm::CodePart::Chassis,0x210,0x211);
 Run(a,b);std::printf("Remote relay: %d checks, %d failures\n",checks,failures);return failures?1:0;
}
