#ifndef _LEGGED_CHEETAH_SDK_LORDIMU_H_
#define _LEGGED_CHEETAH_SDK_LORDIMU_H_

#include "legged_cheetah_sdk/Support/Types.h"

#include <cstdint>

class LordImu {
public:
  bool tryInit(uint32_t port, uint32_t baud_rate);
  void init(uint32_t port, uint32_t baud_rate);
  void run();

public:
  uint32_t invalid_packets = 0;
  uint32_t timeout_packets = 0;
  uint32_t unknown_packets = 0;
  uint32_t good_packets = 0;

  Vec3<float> gyro; // gyroscope
  Vec3<float> acc;  // accelerometer
  Vec4<float> quat; // quaternion
};

#endif // _LEGGED_CHEETAH_SDL_LORDIMU_H_