#include "legged_cheetah_sdk/IMU/LordImu.h"
#include "legged_cheetah_sdk/IMU/mip_sdk_ahrs.h"
#include "legged_cheetah_sdk/IMU/mip_sdk_filter.h"
#include "legged_cheetah_sdk/IMU/mip_types.h"

#include "legged_cheetah_sdk/Support/Math.h"
#include "legged_cheetah_sdk/Support/OrientationTools.h"

#include <mutex>
#include <stdexcept>
#include <thread>

constexpr u32 IMU_PACKET_TIMEOUT_MS = 1000;
constexpr u32 MIP_SDK_GX4_25_IMU_DIRECT_MODE = 0x02;
constexpr u32 MIP_SDK_STANDARD_MODE = 0x01;

mip_interface device_interface;

static LordImu *gLordImu;
static void filter_callback(void *user_ptr, u8 *packet, u16 packet_size,
                            u8 callback_type) {
  (void)user_ptr;
  (void)packet_size;

  printf("Filter callback!\n");

  mip_field_header *field_header;
  u8 *field_data;
  u16 field_offset = 0;
  mip_filter_attitude_quaternion quat;

  //  Mat3<float> R = Mat3<float>::Identity();
  // R << 0, -1, 0, -1, 0, 0, 0, 0, 1;

  switch (callback_type) {
  case MIP_INTERFACE_CALLBACK_VALID_PACKET:
    while (mip_get_next_field(packet, &field_header, &field_data,
                              &field_offset) == MIP_OK) {
      switch (field_header->descriptor) {
      case MIP_FILTER_DATA_ATT_QUATERNION: {
        memcpy(&quat, field_data, sizeof(mip_filter_attitude_quaternion));
        mip_filter_attitude_quaternion_byteswap(&quat);
        gLordImu->quat = Vec4<float>(quat.q);
        Mat3<float> g_R_imu, r_R_imup;

        g_R_imu << 0, 1, 0, 1, 0, 0, 0, 0, -1;
        r_R_imup << 0, 0, 1, 0, -1, 0, 1, 0, 0;

        Vec4<float> ql = ori::rotationMatrixToQuaternion(g_R_imu.transpose());
        Vec4<float> qr = ori::rotationMatrixToQuaternion(r_R_imup);
        gLordImu->quat =
            ori::quatProduct(ql, ori::quatProduct(gLordImu->quat, qr));
        printf("rpy callback: %.3f %.3f %.3f\n", gLordImu->quat[0],
               gLordImu->quat[1], gLordImu->quat[2]);

        gLordImu->good_packets++;
      } break;
      default:
        printf("[Lord IMU] Unknown FILTER packet %d\n",
               field_header->descriptor);
        break;
      }
    }
    break;
  case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
    gLordImu->invalid_packets++;
    break;
  case MIP_INTERFACE_CALLBACK_TIMEOUT:
    gLordImu->timeout_packets++;
    break;
  default:
    gLordImu->unknown_packets++;
    break;
  }
}

static void ahrs_callback(void *user_ptr, u8 *packet, u16 packet_size,
                          u8 callback_type) {
  (void)user_ptr;
  (void)packet_size;

  mip_field_header *field_header;
  u8 *field_data;
  u16 field_offset = 0;
  mip_ahrs_scaled_accel accel;
  mip_ahrs_scaled_gyro gyro;
  mip_filter_attitude_quaternion quat;

  switch (callback_type) {
  case MIP_INTERFACE_CALLBACK_VALID_PACKET:
    // gLordImu->good_packets++;
    while (mip_get_next_field(packet, &field_header, &field_data,
                              &field_offset) == MIP_OK) {
      switch (field_header->descriptor) {
      case MIP_AHRS_DATA_ACCEL_SCALED:
        memcpy(&accel, field_data, sizeof(mip_ahrs_scaled_accel));
        mip_ahrs_scaled_accel_byteswap(&accel);
        gLordImu->acc[0] = 9.81f * accel.scaled_accel[2];
        gLordImu->acc[1] = -9.81f * accel.scaled_accel[1];
        gLordImu->acc[2] = 9.81f * accel.scaled_accel[0];
        gLordImu->good_packets++;
        break;
      case MIP_AHRS_DATA_GYRO_SCALED:
        memcpy(&gyro, field_data, sizeof(mip_ahrs_scaled_gyro));
        mip_ahrs_scaled_gyro_byteswap(&gyro);
        gLordImu->gyro[0] = gyro.scaled_gyro[2];
        gLordImu->gyro[1] = -gyro.scaled_gyro[1];
        gLordImu->gyro[2] = gyro.scaled_gyro[0];
        gLordImu->good_packets++;
        break;

      case MIP_AHRS_DATA_QUATERNION: {
        memcpy(&quat, field_data, sizeof(mip_filter_attitude_quaternion));
        mip_filter_attitude_quaternion_byteswap(&quat);
        gLordImu->quat = Vec4<float>(quat.q);
        Mat3<float> g_R_imu, r_R_imup;

        g_R_imu << 0, 1, 0, 1, 0, 0, 0, 0, -1;
        r_R_imup << 0, 0, 1, 0, -1, 0, 1, 0, 0;

        Vec4<float> ql = ori::rotationMatrixToQuaternion(g_R_imu.transpose());
        Vec4<float> qr = ori::rotationMatrixToQuaternion(r_R_imup);
        gLordImu->quat =
            ori::quatProduct(ql, ori::quatProduct(gLordImu->quat, qr));

        gLordImu->good_packets++;
      } break;
      default:
        printf("[Lord IMU] Unknown AHRS packet %d\n", field_header->descriptor);
        break;
      }
    }
    break;
  case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
    gLordImu->invalid_packets++;
    break;
  case MIP_INTERFACE_CALLBACK_TIMEOUT:
    gLordImu->timeout_packets++;
    break;
  default:
    gLordImu->unknown_packets++;
    break;
  }
}

void setup_streaming() {
  if (mip_interface_add_descriptor_set_callback(
          &device_interface, MIP_FILTER_DATA_SET, nullptr, filter_callback) !=
      MIP_INTERFACE_OK) {
    throw std::runtime_error("failed to set IMU filter callback");
  }

  if (mip_interface_add_descriptor_set_callback(
          &device_interface, MIP_AHRS_DATA_SET, nullptr, ahrs_callback) !=
      MIP_INTERFACE_OK) {
    throw std::runtime_error("failed to set IMU ahrs callback");
  }
}

void LordImu::run() {
  for (u32 i = 0; i < 20; i++)
    mip_interface_update(&device_interface);
  usleep(100);
}

bool LordImu::tryInit(u32 port, u32 baud_rate) {
  try {
    init(port, baud_rate);
  } catch (std::exception &e) {
    printf("[Lord IMU] failed to initialize: %s\n", e.what());
    return false;
  }

  return true;
}

void LordImu::init(u32 port, u32 baud_rate) {
  printf("[Lord IMU] Open port %d, baud rate %d\n", port, baud_rate);

  if (mip_interface_init(port, baud_rate, &device_interface,
                         IMU_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK) {
    throw std::runtime_error("Failed to initialize MIP interface for IMU\n");
  }

  gLordImu = this;

  setup_streaming();
}