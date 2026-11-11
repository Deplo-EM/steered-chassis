#pragma once
#define HW_COMPONENTS_ALGORITHMS_AHRS_MAHONY_HPP_
inline int bmi_ahrs_updates=0;
namespace hello_world::ahrs {
class Mahony {
 public:
  void init(float,float,float) {}
  void update(const float[3],const float[3]) { ++bmi_ahrs_updates; }
  void getEulerAngle(float e[3]) const { e[0]=e[1]=e[2]=0; }
};
}
