#ifndef _LEGGED_CHEETAH_HW_CHEETAHHW_H_
#define _LEGGED_CHEETAH_HW_CHEETAHHW_H_

#include "legged_cheetah_sdk/IMU/LordIMU.h"

#include <legged_hw/LeggedHW.h>

namespace legged {

struct CheetahImu_t {
  double ori[4]; // 	A pointer to the storage of the orientation value: a
                 // quaternion (x,y,z,w)

  double oriCov[9]; // 	A pointer to the storage of the orientation covariance
                    // value: a row major 3x3 matrix about (x,y,z)

  double angularVel[3]; // 	 angular velocity value: a triplet (x,y,z)

  double angularVelCov[9]; // 	 angular velocitycovariance value: a row major
                           // 3x3 matrix about (x,y,z)
  double linearAcc[3];     // 	 linear  acceleration value: a triplet (x,y,z)

  double linearAccCov[9]; // 	 linear  acceleration covariance value: a row
                          // major 3x3  matrix about (x,y,z)
};

struct CheetahJoint_t {
  double pos; // Joint's position.
  double vel; // Joint's vetocity.
  double eff; // Joint's effort (force or torque).

  double posDes;
  double velDes;
  double kp;
  double kd;
  double ff;
};

const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT",
                                                       "RH_FOOT", "LH_FOOT"};

class CheetahHW : public legged::LeggedHW {
public:
  CheetahHW() = default;

public:
  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load
   * urdf of robot. Set up transmission and joint limit. Get configuration of
   * can bus and create data pointer which point to data received from Can bus.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed.
   */
  bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;
  /** \brief Communicate with hardware. Get data, status of robot.
   *
   * Call @ref UNITREE_LEGGED_SDK::UDP::Recv() to get robot's state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time &time, const ros::Duration &period) override;

  /** \brief Comunicate with hardware. Publish command to robot.
   *
   * Propagate joint state to actuator state for the stored
   * transmission. Limit cmd_effort into suitable value. Call @ref
   * UNITREE_LEGGED_SDK::UDP::Recv(). Publish actuator current state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time &time, const ros::Duration &period) override;

private:
  bool setupImu(void);
  bool setupJoints(void);
  bool setupContactSensor(void);

private:
  CheetahImu_t _imuData{};
  CheetahJoint_t _jointData[12]{};
  bool _contactState[4]{};

  std::unique_ptr<LordImu> _imuHandler;
};

} // namespace legged

#endif