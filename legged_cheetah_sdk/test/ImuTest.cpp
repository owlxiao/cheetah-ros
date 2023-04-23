#include "legged_cheetah_sdk/IMU/LordImu.h"
#include "legged_cheetah_sdk/Support/OrientationTools.h"

#include <cstddef>
#include <cstdio>

#include <unistd.h>

int main(int argc, char **argv) {
  std::uint32_t comPort, baudRate;
  if (argc != 3) {
    printf("Usage: imu-test com-port baudrate\n");
    return 1;
  }

  comPort = std::atoi(argv[1]);
  baudRate = std::atoi(argv[2]);

  LordImu imu;

  if (imu.tryInit(comPort, baudRate)) {
    while (true) {
      imu.run();
      Vec3<float> rpy = ori::quatToRPY(imu.quat);
      (void)rpy;
      Vec3<float> acc = imu.acc;
      (void)acc;
      Vec3<float> ang = imu.gyro;
      (void)ang;
      printf("rpy: %.3f %.3f %.3f\n", rpy[0], rpy[1], rpy[2]);
      printf("acc: %.3f %.3f %.3f\n", acc[0], acc[1], acc[2]);
      printf("ang: %.3f %.3f %.3f\n\n", ang[0], ang[1], ang[2]);

      usleep(50000);
    }
  }

  return 0;
}