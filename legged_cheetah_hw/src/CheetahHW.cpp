#include "legged_cheetah_hw/CheetahHW.h"

#include "legged_cheetah_sdk/IMU/LordIMU.h"
#include "legged_cheetah_sdk/Quadruped/Cheetah.h"

#include "legged_common/hardware_interface/ContactSensorInterface.h"
#include "legged_hw/LeggedHW.h"

namespace legged {

bool CheetahHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
  if (!legged::LeggedHW::init(root_nh, robot_hw_nh)) {
    ROS_FATAL("legged::LeggedHW::init error!\n");
    return false;
  }

  _imuHandler = std::make_unique<LordImu>();

  setupImu();
  setupJoints();
  setupContactSensor();

  return true;
}

bool CheetahHW::setupImu(void) {
  if (!_imuHandler->tryInit(0, 460800)) {
    ROS_FATAL("IMU initial error!");
    return false;
  }

  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle(
      "unitree_imu", "unitree_imu", _imuData.ori, _imuData.oriCov,
      _imuData.angularVel, _imuData.angularVelCov, _imuData.linearAcc,
      _imuData.linearAccCov));

  // TODO: How to set up the initial value of imu.
  _imuData.oriCov[0] = 0.0012;
  _imuData.oriCov[4] = 0.0012;
  _imuData.oriCov[8] = 0.0012;

  _imuData.angularVelCov[0] = 0.0004;
  _imuData.angularVelCov[4] = 0.0004;
  _imuData.angularVelCov[8] = 0.0004;

  return true;
}

bool CheetahHW::setupJoints(void) {

  for (const auto &joint : urdfModel_->joints_) {
    // ROS_WARN("%s\n", joint.first.c_str());

    int leg_index = 0;
    int joint_index = 0;
    if (joint.first.find("RF") != std::string::npos) {
      leg_index = FR;
    } else if (joint.first.find("LF") != std::string::npos) {
      leg_index = FL;
    } else if (joint.first.find("RH") != std::string::npos) {
      leg_index = RR;
    } else if (joint.first.find("LH") != std::string::npos) {
      leg_index = RL;
    } else {
      continue;
    }

    if (joint.first.find("HAA") != std::string::npos) {
      joint_index = 0;
    } else if (joint.first.find("HFE") != std::string::npos) {
      joint_index = 1;
    } else if (joint.first.find("KFE") != std::string::npos) {
      joint_index = 2;
    } else {
      continue;
    }

    int index = leg_index * 3 + joint_index;
    hardware_interface::JointStateHandle state_handle(
        joint.first, &_jointData[index].pos, &_jointData[index].vel,
        &_jointData[index].eff);

    jointStateInterface_.registerHandle(state_handle);
    hybridJointInterface_.registerHandle(legged::HybridJointHandle(
        state_handle, &_jointData[index].posDes, &_jointData[index].velDes,
        &_jointData[index].kp, &_jointData[index].kd, &_jointData[index].ff));
  }

  return true;
}

bool CheetahHW::setupContactSensor() {
  for (std::size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contactSensorInterface_.registerHandle(legged::ContactSensorHandle(
        CONTACT_SENSOR_NAMES[i], &_contactState[i]));
  }

  return true;
}

void CheetahHW::read(const ros::Time &time, const ros::Duration &period) {
  // Read Imu data.
  _imuHandler->run();
  _imuData.ori[0] = _imuHandler->quat[1];
  _imuData.ori[1] = _imuHandler->quat[2];
  _imuData.ori[2] = _imuHandler->quat[3];
  _imuData.ori[3] = _imuHandler->quat[0];

  // 3x3 matrix.
  for (std::size_t i = 0; i < 3; ++i) {
    _imuData.angularVel[i] = _imuHandler->gyro[i];
    _imuData.linearAcc[i] = _imuHandler->acc[i];
  }

  ROS_WARN("Orientation: %.3f %.3f %.3f %.3f", _imuData.ori[0], _imuData.ori[1],
           _imuData.ori[2], _imuData.ori[3]);
  ROS_WARN("Angular velocity: %.3f %.3f %.3f", _imuData.angularVel[1],
           _imuData.angularVel[2], _imuData.angularVel[2]);
  ROS_WARN("Linear acceleration: %.3f %.3f %.3f", _imuData.linearAcc[0],
           _imuData.linearAcc[1], _imuData.linearAcc[2]);

  std::vector<std::string> names = hybridJointInterface_.getNames();
  for (const auto &name : names) {
    HybridJointHandle handle = hybridJointInterface_.getHandle(name);
    handle.setFeedforward(0.);
    handle.setVelocityDesired(0.);
    handle.setKd(3.);
  }
}

void CheetahHW::write(const ros::Time &time, const ros::Duration &period) {
  return;
}

} // namespace legged