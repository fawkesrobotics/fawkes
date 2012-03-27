
/***************************************************************************
 *  dcm_thread.cpp - Provide NaoQi DCM to Fawkes
 *
 *  Created: Tue May 31 15:00:54 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "dcm_thread.h"
#include "motion_utils.h"

#include <alproxies/allauncherproxy.h>
#include <alproxies/dcmproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alcore/alerror.h>
#include <almemoryfastaccess/almemoryfastaccess.h>

#include <boost/bind.hpp>

#include <interfaces/NaoJointStiffnessInterface.h>
#include <interfaces/NaoSensorInterface.h>

using namespace fawkes;

enum SensorType {  HEAD_PITCH = 0, HEAD_YAW,
		   L_SHOULDER_PITCH, L_SHOULDER_ROLL, L_ELBOW_YAW, L_ELBOW_ROLL,
		   L_WRIST_YAW, L_HAND,
		   L_HIP_YAW_PITCH, L_HIP_ROLL, L_HIP_PITCH, L_KNEE_PITCH,
		   L_ANKLE_PITCH, L_ANKLE_ROLL,
		   R_SHOULDER_PITCH, R_SHOULDER_ROLL, R_ELBOW_YAW, R_ELBOW_ROLL,
		   R_WRIST_YAW, R_HAND,
		   R_HIP_YAW_PITCH, R_HIP_ROLL, R_HIP_PITCH, R_KNEE_PITCH,
		   R_ANKLE_PITCH, R_ANKLE_ROLL,
		   STIFF_HEAD_PITCH, STIFF_HEAD_YAW,
		   STIFF_L_SHOULDER_PITCH, STIFF_L_SHOULDER_ROLL,
		   STIFF_L_ELBOW_YAW, STIFF_L_ELBOW_ROLL,
		   STIFF_L_WRIST_YAW, STIFF_L_HAND,
		   STIFF_L_HIP_YAW_PITCH, STIFF_L_HIP_ROLL, STIFF_L_HIP_PITCH,
		   STIFF_L_KNEE_PITCH, STIFF_L_ANKLE_PITCH, STIFF_L_ANKLE_ROLL,
		   STIFF_R_SHOULDER_PITCH, STIFF_R_SHOULDER_ROLL,
		   STIFF_R_ELBOW_YAW, STIFF_R_ELBOW_ROLL,
		   STIFF_R_WRIST_YAW, STIFF_R_HAND,
		   STIFF_R_HIP_YAW_PITCH, STIFF_R_HIP_ROLL, STIFF_R_HIP_PITCH,
		   STIFF_R_KNEE_PITCH, STIFF_R_ANKLE_PITCH, STIFF_R_ANKLE_ROLL,
		   ACC_X, ACC_Y, ACC_Z, GYR_X, GYR_Y, GYR_REF, ANGLE_X, ANGLE_Y,
		   L_FSR_FL, L_FSR_FR, L_FSR_RL, L_FSR_RR,
		   R_FSR_FL, R_FSR_FR, R_FSR_RL, R_FSR_RR,
		   L_COP_X, L_COP_Y, L_TOTAL_WEIGHT, R_COP_X, R_COP_Y,
		   R_TOTAL_WEIGHT,
                   ULTRASONIC_DIRECTION, ULTRASONIC_DISTANCE,
                   ULTRASONIC_DISTANCE_LEFT_0, ULTRASONIC_DISTANCE_LEFT_1,
                   ULTRASONIC_DISTANCE_LEFT_2, ULTRASONIC_DISTANCE_LEFT_3,
                   ULTRASONIC_DISTANCE_RIGHT_0, ULTRASONIC_DISTANCE_RIGHT_1,
                   ULTRASONIC_DISTANCE_RIGHT_2, ULTRASONIC_DISTANCE_RIGHT_3,
		   L_FOOT_BUMPER_L, L_FOOT_BUMPER_R,
		   R_FOOT_BUMPER_L, R_FOOT_BUMPER_R,
		   HEAD_TOUCH_FRONT, HEAD_TOUCH_MIDDLE, HEAD_TOUCH_REAR,
		   CHEST_BUTTON, BATTERY_CHARGE,
		   SensorTypeN};

enum StiffnessJoint { STIFFJ_HEAD_PITCH = 0, STIFFJ_HEAD_YAW,
		      STIFFJ_L_SHOULDER_PITCH, STIFFJ_L_SHOULDER_ROLL,
		      STIFFJ_L_ELBOW_YAW, STIFFJ_L_ELBOW_ROLL,
		      STIFFJ_L_HIP_YAW_PITCH, STIFFJ_L_HIP_ROLL,
		      STIFFJ_L_HIP_PITCH, STIFFJ_L_KNEE_PITCH,
		      STIFFJ_L_ANKLE_PITCH, STIFFJ_L_ANKLE_ROLL,
		      STIFFJ_R_SHOULDER_PITCH, STIFFJ_R_SHOULDER_ROLL,
		      STIFFJ_R_ELBOW_YAW, STIFFJ_R_ELBOW_ROLL,
		      STIFFJ_R_HIP_YAW_PITCH, STIFFJ_R_HIP_ROLL,
		      STIFFJ_R_HIP_PITCH, STIFFJ_R_KNEE_PITCH,
		      STIFFJ_R_ANKLE_PITCH, STIFFJ_R_ANKLE_ROLL,
		      STIFFJ_L_WRIST_YAW, STIFFJ_L_HAND,
		      STIFFJ_R_WRIST_YAW, STIFFJ_R_HAND,
		      StiffnessJointN };

#define ACCELEROMETER_G_FACTOR 56.

// Clipping as suggested by Aldebaran at RoboCup 2008, Suzhou
// Changed head yaw to +/- 1.2 according to Nao v3 red book
#define HEAD_YAW_MIN          -1.2
#define HEAD_YAW_MAX           1.2
// New values after RoboCup Workshop 2009 at Aldebaran, Paris
#define L_SHOULDER_PITCH_MIN  -1.7
#define L_SHOULDER_PITCH_MAX   1.7
#define R_SHOULDER_PITCH_MIN  -1.7
#define R_SHOULDER_PITCH_MAX   1.7

#define CLIP_VALUE(V, value) clip_value(value, V ## _MIN, V ## _MAX)

/** Clip value to given constraints.
 * @param value value to clop
 * @param min minimum value
 * @param max maximum value
 * @return clipped value
 */
static inline float
clip_value(float value, float min, float max)
{
  if (value < min) value = min;
  if (value > max) value = max;
  return value;
}


/** Thread to write data at full DCM frequency.
 * This thread is woken up by the DCM callback and publishes new data
 * if there is a reader for any of the high frequency interfaces. It
 * is also responsible for processing incoming commands.
 */
class NaoQiDCMThread::HighFreqThread : public Thread
{
 public:
  /** Constructor.
   * @param parent parent NaoQiDCMThread to call.
   */
  HighFreqThread(NaoQiDCMThread *parent)
    : Thread("NaoQiDCMThread::HighFreqThread", Thread::OPMODE_WAITFORWAKEUP),
      __parent(parent)
  {
    set_coalesce_wakeups(true);
  }

  virtual void loop()
  {
    __parent->read_values();
    if ( (__parent->__joint_pos_highfreq_if->num_readers() > 0) ||
	 (__parent->__joint_stiffness_highfreq_if->num_readers() > 0) ||
	 (__parent->__sensor_if->num_readers() > 0) )
    {
      __parent->update_interfaces(__parent->__joint_pos_highfreq_if,
				  __parent->__joint_stiffness_highfreq_if,
				  __parent->__sensor_highfreq_if);
    }

    __parent->process_messages();
  }


 private:
  NaoQiDCMThread *__parent;
};



/** @class NaoQiDCMThread "dcm_thread.h"
 * Thread to provide DCM to Fawkes.
 * This thread opens a DCM proxy and updates information about hardware
 * in the blackboard and provides basic setting functionality to move
 * specific servos.
 *
 * The DCM thread writes to two sets of interfaces, at a high and a lower
 * frequency. The high frequency data is written if there is a reader at
 * each DCM callback, which results in a frequency of about 100Hz. This
 * should only be used if necessary, for example for custom motion pattern
 * generating plugins. Otherwise, and especially if in a thread hooked
 * into the main loop, use the lower frequency interfaces. These are
 * written during the sensor hook.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
NaoQiDCMThread::NaoQiDCMThread()
  : Thread("NaoQiDCMThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE)
{
}


/** Destructor. */
NaoQiDCMThread::~NaoQiDCMThread()
{
}


void
NaoQiDCMThread::init()
{
  // Is the DCM running ?
  try {
    AL::ALPtr<AL::ALLauncherProxy> launcher(new AL::ALLauncherProxy(naoqi_broker));
    bool is_dcm_available = launcher->isModulePresent("DCM");
    bool is_almotion_available = launcher->isModulePresent("ALMotion");

    if (! is_dcm_available) {
      throw Exception("DCMThread: NaoQi DCM is not available");
    }
    if (! is_almotion_available) {
      throw Exception("DCMThread: ALMotion is not available");
    }
  } catch (AL::ALError& e) {
    throw Exception("Checking module availability failed: %s",
		    e.toString().c_str());
  }

  __dcm = naoqi_broker->getDcmProxy();
  __almotion = naoqi_broker->getMotionProxy();

  try {
    AL::ALPtr<AL::ALMemoryProxy> almemory = naoqi_broker->getMemoryProxy();
    std::string version = almemory->getData("RobotConfig/Body/BaseVersion", 0);
    unsigned int num_joints = __almotion->getJointNames("Body").size();
    if (num_joints == 26) {
      __robot_type = NaoJointPositionInterface::ROBOTYPE_ACADEMIC;
    } else {
      __robot_type = NaoJointPositionInterface::ROBOTYPE_ROBOCUP;
    }
    __robot_version[2] = __robot_version[3] = 0;
    if (version[0] == 'V')  version = version.substr(1);
    std::string::size_type pos;
    if ((pos = version.find_first_of(".")) != std::string::npos) {
      std::string version_major = version.substr(0, pos);
      std::string version_minor = version.substr(pos+1);
      __robot_version[0] = atoi(version_major.c_str());
      __robot_version[1] = atoi(version_minor.c_str());
    }
    __usboard_version =
      almemory->getData("Device/DeviceList/USBoard/ProgVersion", 0);
  } catch (AL::ALError &e) {
    throw Exception("Retrieving robot info failed: %s", e.toString().c_str());
  }

  __memfa.reset(new AL::ALMemoryFastAccess());

  std::string prefix = "Device/SubDeviceList/";

  // Initialize fast memory access
  std::vector<std::string> keys;
  keys.resize(SensorTypeN);
  __values.resize(SensorTypeN);

  keys[HEAD_PITCH]       = prefix + "HeadPitch/Position/Sensor/Value";
  keys[HEAD_YAW]         = prefix + "HeadYaw/Position/Sensor/Value";
  keys[L_SHOULDER_PITCH] = prefix + "LShoulderPitch/Position/Sensor/Value";
  keys[L_SHOULDER_ROLL]  = prefix + "LShoulderRoll/Position/Sensor/Value";
  keys[L_ELBOW_YAW]      = prefix + "LElbowYaw/Position/Sensor/Value";
  keys[L_ELBOW_ROLL]     = prefix + "LElbowRoll/Position/Sensor/Value";
  keys[L_WRIST_YAW]      = prefix + "LWristYaw/Position/Sensor/Value";
  keys[L_HAND]           = prefix + "LHand/Position/Sensor/Value";
  keys[L_HIP_YAW_PITCH]  = prefix + "LHipYawPitch/Position/Sensor/Value";
  keys[L_HIP_ROLL]       = prefix + "LHipRoll/Position/Sensor/Value";
  keys[L_HIP_PITCH]      = prefix + "LHipPitch/Position/Sensor/Value";
  keys[L_KNEE_PITCH]     = prefix + "LKneePitch/Position/Sensor/Value";
  keys[L_ANKLE_PITCH]    = prefix + "LAnklePitch/Position/Sensor/Value";
  keys[L_ANKLE_ROLL]     = prefix + "LAnkleRoll/Position/Sensor/Value";

  keys[R_SHOULDER_PITCH] = prefix + "RShoulderPitch/Position/Sensor/Value";
  keys[R_SHOULDER_ROLL]  = prefix + "RShoulderRoll/Position/Sensor/Value";
  keys[R_ELBOW_YAW]      = prefix + "RElbowYaw/Position/Sensor/Value";
  keys[R_ELBOW_ROLL]     = prefix + "RElbowRoll/Position/Sensor/Value";
  keys[R_WRIST_YAW]      = prefix + "RWristYaw/Position/Sensor/Value";
  keys[R_HAND]           = prefix + "RHand/Position/Sensor/Value";
  keys[R_HIP_YAW_PITCH]  = prefix + "RHipYawPitch/Position/Sensor/Value";
  keys[R_HIP_ROLL]       = prefix + "RHipRoll/Position/Sensor/Value";
  keys[R_HIP_PITCH]      = prefix + "RHipPitch/Position/Sensor/Value";
  keys[R_KNEE_PITCH]     = prefix + "RKneePitch/Position/Sensor/Value";
  keys[R_ANKLE_PITCH]    = prefix + "RAnklePitch/Position/Sensor/Value";
  keys[R_ANKLE_ROLL]     = prefix + "RAnkleRoll/Position/Sensor/Value";

  keys[STIFF_HEAD_PITCH]       = prefix + "HeadPitch/Hardness/Actuator/Value";
  keys[STIFF_HEAD_YAW]         = prefix + "HeadYaw/Hardness/Actuator/Value";
  keys[STIFF_L_SHOULDER_PITCH] = prefix + "LShoulderPitch/Hardness/Actuator/Value";
  keys[STIFF_L_SHOULDER_ROLL]  = prefix + "LShoulderRoll/Hardness/Actuator/Value";
  keys[STIFF_L_ELBOW_YAW]      = prefix + "LElbowYaw/Hardness/Actuator/Value";
  keys[STIFF_L_ELBOW_ROLL]     = prefix + "LElbowRoll/Hardness/Actuator/Value";
  keys[STIFF_L_WRIST_YAW]      = prefix + "LWristYaw/Hardness/Actuator/Value";
  keys[STIFF_L_HAND]           = prefix + "LHand/Hardness/Actuator/Value";
  keys[STIFF_L_HIP_YAW_PITCH]  = prefix + "LHipYawPitch/Hardness/Actuator/Value";
  keys[STIFF_L_HIP_ROLL]       = prefix + "LHipRoll/Hardness/Actuator/Value";
  keys[STIFF_L_HIP_PITCH]      = prefix + "LHipPitch/Hardness/Actuator/Value";
  keys[STIFF_L_KNEE_PITCH]     = prefix + "LKneePitch/Hardness/Actuator/Value";
  keys[STIFF_L_ANKLE_PITCH]    = prefix + "LAnklePitch/Hardness/Actuator/Value";
  keys[STIFF_L_ANKLE_ROLL]     = prefix + "LAnkleRoll/Hardness/Actuator/Value";

  keys[STIFF_R_SHOULDER_PITCH] = prefix + "RShoulderPitch/Hardness/Actuator/Value";
  keys[STIFF_R_SHOULDER_ROLL]  = prefix + "RShoulderRoll/Hardness/Actuator/Value";
  keys[STIFF_R_ELBOW_YAW]      = prefix + "RElbowYaw/Hardness/Actuator/Value";
  keys[STIFF_R_ELBOW_ROLL]     = prefix + "RElbowRoll/Hardness/Actuator/Value";
  keys[STIFF_R_WRIST_YAW]      = prefix + "RWristYaw/Hardness/Actuator/Value";
  keys[STIFF_R_HAND]           = prefix + "RHand/Hardness/Actuator/Value";
  keys[STIFF_R_HIP_YAW_PITCH]  = prefix + "RHipYawPitch/Hardness/Actuator/Value";
  keys[STIFF_R_HIP_ROLL]       = prefix + "RHipRoll/Hardness/Actuator/Value";
  keys[STIFF_R_HIP_PITCH]      = prefix + "RHipPitch/Hardness/Actuator/Value";
  keys[STIFF_R_KNEE_PITCH]     = prefix + "RKneePitch/Hardness/Actuator/Value";
  keys[STIFF_R_ANKLE_PITCH]    = prefix + "RAnklePitch/Hardness/Actuator/Value";
  keys[STIFF_R_ANKLE_ROLL]     = prefix + "RAnkleRoll/Hardness/Actuator/Value";

  // Inertial sensors
  keys[ACC_X]   = prefix + "InertialSensor/AccX/Sensor/Value";
  keys[ACC_Y]   = prefix + "InertialSensor/AccY/Sensor/Value";
  keys[ACC_Z]   = prefix + "InertialSensor/AccZ/Sensor/Value";
  keys[GYR_X]   = prefix + "InertialSensor/GyrX/Sensor/Value";
  keys[GYR_Y]   = prefix + "InertialSensor/GyrY/Sensor/Value";
  keys[GYR_REF] = prefix + "InertialSensor/GyrRef/Sensor/Value";
  keys[ANGLE_X] = prefix + "InertialSensor/AngleX/Sensor/Value";
  keys[ANGLE_Y] = prefix + "InertialSensor/AngleY/Sensor/Value";

  // FSR sensors
  keys[L_FSR_FL]     = prefix + "LFoot/FSR/FrontLeft/Sensor/Value";
  keys[L_FSR_FR]     = prefix + "LFoot/FSR/FrontRight/Sensor/Value";
  keys[L_FSR_RL]     = prefix + "LFoot/FSR/RearLeft/Sensor/Value";
  keys[L_FSR_RR]     = prefix + "LFoot/FSR/RearRight/Sensor/Value";
  keys[R_FSR_FL]     = prefix + "RFoot/FSR/FrontLeft/Sensor/Value";
  keys[R_FSR_FR]     = prefix + "RFoot/FSR/FrontRight/Sensor/Value";
  keys[R_FSR_RL]     = prefix + "RFoot/FSR/RearLeft/Sensor/Value";
  keys[R_FSR_RR]     = prefix + "RFoot/FSR/RearRight/Sensor/Value";
  keys[L_COP_X]        = prefix + "LFoot/FSR/CenterOfPressure/X/Sensor/Value";
  keys[L_COP_Y]        = prefix + "LFoot/FSR/CenterOfPressure/Y/Sensor/Value";
  keys[L_TOTAL_WEIGHT] = prefix + "LFoot/FSR/TotalWeight/Sensor/Value";
  keys[R_COP_X]        = prefix + "RFoot/FSR/CenterOfPressure/X/Sensor/Value";
  keys[R_COP_Y]        = prefix + "RFoot/FSR/CenterOfPressure/Y/Sensor/Value";
  keys[R_TOTAL_WEIGHT] = prefix + "RFoot/FSR/TotalWeight/Sensor/Value";

  // Ultrasonic
  keys[ULTRASONIC_DIRECTION]        = prefix + "US/Actuator/Value";
  keys[ULTRASONIC_DISTANCE]         = prefix + "US/Sensor/Value";

  keys[ULTRASONIC_DISTANCE_LEFT_0]  = prefix + "US/Left/Sensor/Value";
  keys[ULTRASONIC_DISTANCE_LEFT_1]  = prefix + "US/Left/Sensor/Value1";
  keys[ULTRASONIC_DISTANCE_LEFT_2]  = prefix + "US/Left/Sensor/Value2";
  keys[ULTRASONIC_DISTANCE_LEFT_3]  = prefix + "US/Left/Sensor/Value3";

  keys[ULTRASONIC_DISTANCE_RIGHT_0] = prefix + "US/Right/Sensor/Value";
  keys[ULTRASONIC_DISTANCE_RIGHT_1] = prefix + "US/Right/Sensor/Value1";
  keys[ULTRASONIC_DISTANCE_RIGHT_2] = prefix + "US/Right/Sensor/Value2";
  keys[ULTRASONIC_DISTANCE_RIGHT_3] = prefix + "US/Right/Sensor/Value3";

  // Bumpers and Buttons
  keys[L_FOOT_BUMPER_L]     = prefix + "LFoot/Bumper/Left/Sensor/Value";
  keys[L_FOOT_BUMPER_R]     = prefix + "LFoot/Bumper/Right/Sensor/Value";
  keys[R_FOOT_BUMPER_L]     = prefix + "RFoot/Bumper/Left/Sensor/Value";
  keys[R_FOOT_BUMPER_R]     = prefix + "RFoot/Bumper/Right/Sensor/Value";

  keys[HEAD_TOUCH_FRONT]     = prefix + "Head/Touch/Front/Sensor/Value";
  keys[HEAD_TOUCH_MIDDLE]    = prefix + "Head/Touch/Middle/Sensor/Value";
  keys[HEAD_TOUCH_REAR]      = prefix + "Head/Touch/Rear/Sensor/Value";

  keys[CHEST_BUTTON]     = prefix + "ChestBoard/Button/Sensor/Value";

  // Battery
  keys[BATTERY_CHARGE]     = prefix + "Battery/Charge/Sensor/Value";

  try {
    __memfa->ConnectToVariables(naoqi_broker, keys, false);
  } catch (AL::ALError &e) {
    throw Exception("Failed to setup fast memory access: %s",
		    e.toString().c_str());
  }


  // Setup alias for setting stiffness, reuse fastmem keys vector
  if (__robot_type == NaoJointPositionInterface::ROBOTYPE_ROBOCUP) {
    __alljoint_names.arraySetSize(22);
  } else {
    __alljoint_names.arraySetSize(26);
    __alljoint_names[STIFFJ_L_WRIST_YAW]      = "LWristYaw";
    __alljoint_names[STIFFJ_L_HAND]           = "LHand";
    __alljoint_names[STIFFJ_R_WRIST_YAW]      = "RWristYaw";
    __alljoint_names[STIFFJ_R_HAND]           = "RHand";
  }
  __alljoint_names[STIFFJ_HEAD_PITCH]       = "HeadPitch";
  __alljoint_names[STIFFJ_HEAD_YAW]         = "HeadYaw";
  __alljoint_names[STIFFJ_L_SHOULDER_PITCH] = "LShoulderPitch";
  __alljoint_names[STIFFJ_L_SHOULDER_ROLL]  = "LShoulderRoll";
  __alljoint_names[STIFFJ_L_ELBOW_YAW]      = "LElbowYaw";
  __alljoint_names[STIFFJ_L_ELBOW_ROLL]     = "LElbowRoll";
  __alljoint_names[STIFFJ_L_HIP_YAW_PITCH]  = "LHipYawPitch";
  __alljoint_names[STIFFJ_L_HIP_ROLL]       = "LHipRoll";
  __alljoint_names[STIFFJ_L_HIP_PITCH]      = "LHipPitch";
  __alljoint_names[STIFFJ_L_KNEE_PITCH]     = "LKneePitch";
  __alljoint_names[STIFFJ_L_ANKLE_PITCH]    = "LAnklePitch";
  __alljoint_names[STIFFJ_L_ANKLE_ROLL]     = "LAnkleRoll";

  __alljoint_names[STIFFJ_R_SHOULDER_PITCH] = "RShoulderPitch";
  __alljoint_names[STIFFJ_R_SHOULDER_ROLL]  = "RShoulderRoll";
  __alljoint_names[STIFFJ_R_ELBOW_YAW]      = "RElbowYaw";
  __alljoint_names[STIFFJ_R_ELBOW_ROLL]     = "RElbowRoll";
  __alljoint_names[STIFFJ_R_HIP_YAW_PITCH]  = "RHipYawPitch";
  __alljoint_names[STIFFJ_R_HIP_ROLL]       = "RHipRoll";
  __alljoint_names[STIFFJ_R_HIP_PITCH]      = "RHipPitch";
  __alljoint_names[STIFFJ_R_KNEE_PITCH]     = "RKneePitch";
  __alljoint_names[STIFFJ_R_ANKLE_PITCH]    = "RAnklePitch";
  __alljoint_names[STIFFJ_R_ANKLE_ROLL]     = "RAnkleRoll";

  try {
    AL::ALValue setJointStiffnessAlias;
    // Alias for all joint stiffness
    setJointStiffnessAlias.clear();
    setJointStiffnessAlias.arraySetSize(2);
    setJointStiffnessAlias[0] = std::string("setJointStiffness");
    setJointStiffnessAlias[1].arraySetSize(26);

    // stiffness list
    int offset = STIFF_HEAD_PITCH - HEAD_PITCH;
    for (int i = HEAD_PITCH; i <= R_ANKLE_ROLL; ++i) {
      setJointStiffnessAlias[1][i] = keys[i + offset];
    }

    __dcm->createAlias(setJointStiffnessAlias);
  } catch (AL::ALError &e) {
    __memfa.reset();
    throw Exception("Failed to create SetJointStiffness alias: %s",
		    e.toString().c_str());
  }

  __joint_pos_if =
    blackboard->open_for_writing<NaoJointPositionInterface>("Nao Joint Positions");
  __joint_stiffness_if =
    blackboard->open_for_writing<NaoJointStiffnessInterface>
                                ("Nao Joint Stiffness");
  __sensor_if =
    blackboard->open_for_writing<NaoSensorInterface>("Nao Sensors");

  __joint_pos_highfreq_if =
    blackboard->open_for_writing<NaoJointPositionInterface>
                                ("Nao Joint Positions HF");
  __joint_stiffness_highfreq_if =
    blackboard->open_for_writing<NaoJointStiffnessInterface>
                                ("Nao Joint Stiffness HF");
  __sensor_highfreq_if =
    blackboard->open_for_writing<NaoSensorInterface>("Nao Sensors HF");


  // Get all values from ALMemory using fastaccess
  __dcm_time = __dcm->getTime(0);
  __memfa->GetValues(__values);

  __joint_pos_if->set_robot_type(__robot_type);
  __joint_pos_if->set_robot_version(__robot_version);
  __joint_pos_highfreq_if->set_robot_type(__robot_type);
  __joint_pos_highfreq_if->set_robot_version(__robot_version);

  __sensor_if->set_ultrasonic_direction(NaoSensorInterface::USD_NONE);
  __sensor_highfreq_if->set_ultrasonic_direction(NaoSensorInterface::USD_NONE);

  // Write once the current data on startup
  update_interfaces(__joint_pos_if, __joint_stiffness_if, __sensor_if);
  update_interfaces(__joint_pos_highfreq_if, __joint_stiffness_highfreq_if,
		    __sensor_highfreq_if);

  __highfreq_thread = new NaoQiDCMThread::HighFreqThread(this);
  __highfreq_thread->start();

  __dcm_sigconn =
    __dcm->getGenericProxy()->getModule()->
    atPostProcess(boost::bind(&NaoQiDCMThread::dcm_callback, this));
  
  /*
  AL::ALValue cmd;
  cmd.arraySetSize(3);
  cmd[0] = std::string("setJointStiffness");
  cmd[1] = std::string("Merge");
  cmd[2].arraySetSize(1);
  cmd[2][0].arraySetSize(2);
  cmd[2][0][0] = 1.0;
  cmd[2][0][1] = __dcm_time + 500;
  __dcm->set(cmd);

  send_command("HeadPitch/Position/Sensor/Value", 0.4, 1000);
  */
}


void
NaoQiDCMThread::dcm_callback()
{
  __highfreq_thread->wakeup();
}

void
NaoQiDCMThread::finalize()
{
  __dcm_sigconn.disconnect();

  __highfreq_thread->cancel();
  __highfreq_thread->join();
  delete __highfreq_thread;

  blackboard->close(__joint_pos_if);
  blackboard->close(__joint_stiffness_if);
  blackboard->close(__sensor_if);
  blackboard->close(__joint_pos_highfreq_if);
  blackboard->close(__joint_stiffness_highfreq_if);
  blackboard->close(__sensor_highfreq_if);
  __joint_pos_if = NULL;
  __joint_stiffness_if = NULL;
  __sensor_if = NULL;
  __joint_pos_highfreq_if = NULL;
  __joint_stiffness_highfreq_if = NULL;
  __sensor_highfreq_if = NULL;

  __memfa.reset();
  __dcm.reset();
}


void
NaoQiDCMThread::read_values()
{
  __values.lock();
  __dcm_time = __dcm->getTime(0);
  __memfa->GetValues(__values);
  __values.unlock();
}


void
NaoQiDCMThread::loop()
{
  update_interfaces(__joint_pos_if, __joint_stiffness_if, __sensor_if);
}

void
NaoQiDCMThread::update_interfaces(NaoJointPositionInterface *joint_pos_if,
				  NaoJointStiffnessInterface *joint_stiffness_if,
				  NaoSensorInterface *sensor_if)
{
  // Joint Position
  // Head
  joint_pos_if->set_head_yaw(__values[HEAD_YAW]);
  joint_pos_if->set_head_pitch(__values[HEAD_PITCH]);
  // Arms
  joint_pos_if->set_l_shoulder_pitch(__values[L_SHOULDER_PITCH]);
  joint_pos_if->set_l_shoulder_roll(__values[L_SHOULDER_ROLL]);
  joint_pos_if->set_l_elbow_yaw(__values[L_ELBOW_YAW]);
  joint_pos_if->set_l_elbow_roll(__values[L_ELBOW_ROLL]);
  joint_pos_if->set_l_wrist_yaw(__values[L_WRIST_YAW]);
  joint_pos_if->set_l_hand(__values[L_HAND]);
  joint_pos_if->set_r_shoulder_pitch(__values[R_SHOULDER_PITCH]);
  joint_pos_if->set_r_shoulder_roll(__values[R_SHOULDER_ROLL]);
  joint_pos_if->set_r_elbow_yaw(__values[R_ELBOW_YAW]);
  joint_pos_if->set_r_elbow_roll(__values[R_ELBOW_ROLL]);
  joint_pos_if->set_r_wrist_yaw(__values[R_WRIST_YAW]);
  joint_pos_if->set_r_hand(__values[R_HAND]);
  // Hip
  joint_pos_if->set_l_hip_yaw_pitch(__values[L_HIP_YAW_PITCH]);
  joint_pos_if->set_l_hip_pitch(__values[L_HIP_PITCH]);
  joint_pos_if->set_l_hip_roll(__values[L_HIP_ROLL]);
  joint_pos_if->set_r_hip_yaw_pitch(__values[R_HIP_YAW_PITCH]);
  joint_pos_if->set_r_hip_pitch(__values[R_HIP_PITCH]);
  joint_pos_if->set_r_hip_roll(__values[R_HIP_ROLL]);
  // Knees
  joint_pos_if->set_l_knee_pitch(__values[L_KNEE_PITCH]);
  joint_pos_if->set_r_knee_pitch(__values[R_KNEE_PITCH]);
  // Feet
  joint_pos_if->set_l_ankle_pitch(__values[L_ANKLE_PITCH]);
  joint_pos_if->set_l_ankle_roll(__values[L_ANKLE_ROLL]);
  joint_pos_if->set_r_ankle_pitch(__values[R_ANKLE_PITCH]);
  joint_pos_if->set_r_ankle_roll(__values[R_ANKLE_ROLL]);

  joint_pos_if->set_time(__dcm_time);

  // Joint Stiffness
  // Head
  joint_stiffness_if->set_head_yaw(__values[STIFF_HEAD_YAW]);
  joint_stiffness_if->set_head_pitch(__values[STIFF_HEAD_PITCH]);
  // Arms
  joint_stiffness_if->set_l_shoulder_pitch(__values[STIFF_L_SHOULDER_PITCH]);
  joint_stiffness_if->set_l_shoulder_roll(__values[STIFF_L_SHOULDER_ROLL]);
  joint_stiffness_if->set_l_elbow_yaw(__values[STIFF_L_ELBOW_YAW]);
  joint_stiffness_if->set_l_elbow_roll(__values[STIFF_L_ELBOW_ROLL]);
  joint_stiffness_if->set_l_wrist_yaw(__values[STIFF_L_WRIST_YAW]);
  joint_stiffness_if->set_l_hand(__values[STIFF_L_HAND]);
  joint_stiffness_if->set_r_shoulder_pitch(__values[STIFF_R_SHOULDER_PITCH]);
  joint_stiffness_if->set_r_shoulder_roll(__values[STIFF_R_SHOULDER_ROLL]);
  joint_stiffness_if->set_r_elbow_yaw(__values[STIFF_R_ELBOW_YAW]);
  joint_stiffness_if->set_r_elbow_roll(__values[STIFF_R_ELBOW_ROLL]);
  joint_stiffness_if->set_r_wrist_yaw(__values[STIFF_R_WRIST_YAW]);
  joint_stiffness_if->set_r_hand(__values[STIFF_R_HAND]);
  // Hip
  joint_stiffness_if->set_l_hip_yaw_pitch(__values[STIFF_L_HIP_YAW_PITCH]);
  joint_stiffness_if->set_l_hip_pitch(__values[STIFF_L_HIP_PITCH]);
  joint_stiffness_if->set_l_hip_roll(__values[STIFF_L_HIP_ROLL]);
  // RHipYawPitch stiffness is always 0, copy from RYawPitch
  joint_stiffness_if->set_r_hip_yaw_pitch(__values[STIFF_L_HIP_YAW_PITCH]);
  joint_stiffness_if->set_r_hip_pitch(__values[STIFF_R_HIP_PITCH]);
  joint_stiffness_if->set_r_hip_roll(__values[STIFF_R_HIP_ROLL]);
  // Knees
  joint_stiffness_if->set_l_knee_pitch(__values[STIFF_L_KNEE_PITCH]);
  joint_stiffness_if->set_r_knee_pitch(__values[STIFF_R_KNEE_PITCH]);
  // Feet
  joint_stiffness_if->set_l_ankle_pitch(__values[STIFF_L_ANKLE_PITCH]);
  joint_stiffness_if->set_l_ankle_roll(__values[STIFF_L_ANKLE_ROLL]);
  joint_stiffness_if->set_r_ankle_pitch(__values[STIFF_R_ANKLE_PITCH]);
  joint_stiffness_if->set_r_ankle_roll(__values[STIFF_R_ANKLE_ROLL]);

  float min_stiffness = 1.;
  for (int i = STIFF_HEAD_YAW; i <= STIFF_R_ANKLE_ROLL; ++i) {
    // ignore wrist and hand on RoboCup version
    if ( (__robot_type == NaoJointPositionInterface::ROBOTYPE_ROBOCUP) &&
	 ( (i == STIFF_L_WRIST_YAW) || (i == STIFF_L_HAND) ||
	   (i == STIFF_R_WRIST_YAW) || (i == STIFF_R_HAND) ) )
      continue;

    // ignore RHipYawPitch stiffness, it's always 0
    if (i == STIFF_R_HIP_YAW_PITCH) continue;

    if (__values[i] < min_stiffness)  min_stiffness = __values[i];
  }
  joint_stiffness_if->set_minimum(min_stiffness);

  // Sensors
  // FSRs
  sensor_if->set_l_fsr_fl(__values[L_FSR_FL]);
  sensor_if->set_l_fsr_fr(__values[L_FSR_FR]);
  sensor_if->set_l_fsr_rl(__values[L_FSR_RL]);
  sensor_if->set_l_fsr_rr(__values[L_FSR_RR]);
  sensor_if->set_r_fsr_fl(__values[R_FSR_FL]);
  sensor_if->set_r_fsr_fr(__values[R_FSR_FR]);
  sensor_if->set_r_fsr_rl(__values[R_FSR_RL]);
  sensor_if->set_r_fsr_rr(__values[R_FSR_RR]);

  sensor_if->set_l_cop_x(__values[L_COP_X]);
  sensor_if->set_l_cop_y(__values[L_COP_Y]);
  sensor_if->set_l_total_weight(__values[L_TOTAL_WEIGHT]);

  sensor_if->set_r_cop_x(__values[R_COP_X]);
  sensor_if->set_r_cop_y(__values[R_COP_Y]);
  sensor_if->set_r_total_weight(__values[R_TOTAL_WEIGHT]);

  // Buttons and bumpers
  sensor_if->set_chest_button((__values[CHEST_BUTTON] >= 0.5) ? 1 : 0);
  sensor_if->set_head_touch_front((__values[HEAD_TOUCH_FRONT] >= 0.5) ? 1 : 0);
  sensor_if->set_head_touch_middle((__values[HEAD_TOUCH_MIDDLE] >= 0.5) ? 1 : 0);
  sensor_if->set_head_touch_rear((__values[HEAD_TOUCH_REAR] >= 0.5) ? 1 : 0);

  sensor_if->set_l_foot_bumper_l((__values[L_FOOT_BUMPER_L] >= 0.5) ? 1 : 0);
  sensor_if->set_l_foot_bumper_r((__values[L_FOOT_BUMPER_R] >= 0.5) ? 1 : 0);
  sensor_if->set_r_foot_bumper_l((__values[R_FOOT_BUMPER_L] >= 0.5) ? 1 : 0);
  sensor_if->set_r_foot_bumper_r((__values[R_FOOT_BUMPER_R] >= 0.5) ? 1 : 0);

  // Inertial measurement unit
  sensor_if->set_accel_x(__values[ACC_X] / ACCELEROMETER_G_FACTOR);
  sensor_if->set_accel_y(__values[ACC_Y] / ACCELEROMETER_G_FACTOR);
  sensor_if->set_accel_z(__values[ACC_Z] / ACCELEROMETER_G_FACTOR);

  sensor_if->set_gyro_x(__values[GYR_X]);
  sensor_if->set_gyro_y(__values[GYR_Y]);
  sensor_if->set_gyro_ref(__values[GYR_REF]);

  sensor_if->set_angle_x(__values[ANGLE_X]);
  sensor_if->set_angle_y(__values[ANGLE_Y]);

  // Ultrasonic sound
  NaoSensorInterface::UltrasonicDirection us_dir =
    sensor_if->ultrasonic_direction();
  switch (us_dir) {
  case NaoSensorInterface::USD_LEFT_LEFT:
  case NaoSensorInterface::USD_RIGHT_LEFT:
    {
      float us_left[4] = {__values[ULTRASONIC_DISTANCE], 0, 0, 0};
      sensor_if->set_ultrasonic_distance_left(us_left);

      float us_right[4] = {0, 0, 0, 0};
      sensor_if->set_ultrasonic_distance_right(us_right);
    }
    break;
  case NaoSensorInterface::USD_LEFT_RIGHT:
  case NaoSensorInterface::USD_RIGHT_RIGHT:
    {
      float us_left[4] = {0, 0, 0, 0};
      sensor_if->set_ultrasonic_distance_left(us_left);

      float us_right[4] = {__values[ULTRASONIC_DISTANCE], 0, 0, 0};
      sensor_if->set_ultrasonic_distance_right(us_right);
    }
    break;

  default:
    {
      float us_left[4] = {__values[ULTRASONIC_DISTANCE_LEFT_0],
                          __values[ULTRASONIC_DISTANCE_LEFT_1],
                          __values[ULTRASONIC_DISTANCE_LEFT_2],
                          __values[ULTRASONIC_DISTANCE_LEFT_3]};
      sensor_if->set_ultrasonic_distance_left(us_left);

      float us_right[4] = {__values[ULTRASONIC_DISTANCE_RIGHT_0],
                           __values[ULTRASONIC_DISTANCE_RIGHT_1],
                           __values[ULTRASONIC_DISTANCE_RIGHT_2],
                           __values[ULTRASONIC_DISTANCE_RIGHT_3]};
      sensor_if->set_ultrasonic_distance_right(us_right);
    }
    break;
  }

  // Battery
  sensor_if->set_battery_charge(__values[BATTERY_CHARGE]);

  // Write to blackboard
  joint_pos_if->write();
  joint_stiffness_if->write();
  sensor_if->write();
}


void
NaoQiDCMThread::process_messages()
{
  // *** Joint position messages
  while (! __joint_pos_if->msgq_empty()) {
    if (NaoJointPositionInterface::SetServoMessage *msg =
	__joint_pos_if->msgq_first_safe(msg))
    {
      send_commands(msg->servo(), "Position", msg->value(), msg->time());
    }

    else if (NaoJointPositionInterface::SetServosMessage *msg =
	     __joint_pos_if->msgq_first_safe(msg))
    {
    }

    else if (NaoJointPositionInterface::MoveServoMessage *msg =
	     __joint_pos_if->msgq_first_safe(msg))
    {
      std::vector<std::string> servos = parse_servo_bitfield(msg->servo());
      std::vector<float> values(servos.size(), msg->value());
      __almotion->setAngles(servos, values, msg->speed());
    }

    else if (NaoJointPositionInterface::MoveServosMessage *msg =
	     __joint_pos_if->msgq_first_safe(msg))
    {
      motion::move_joints(
        __almotion,
        /* head */ msg->head_yaw(), msg->head_pitch(),
        /* l shoulder */ msg->l_shoulder_pitch(), msg->l_shoulder_roll(),
        /* l elbow */ msg->l_elbow_yaw(), msg->l_elbow_roll(),
        /* l wrist/hand */ msg->l_wrist_yaw(), msg->l_hand(),
        /* l hip */ msg->l_hip_yaw_pitch(), msg->l_hip_roll(), msg->l_hip_pitch(),
        /* l knee */ msg->l_knee_pitch(),
        /* l ankle */ msg->l_ankle_pitch(), msg->l_ankle_roll(),
        /* r shoulder */ msg->r_shoulder_pitch(), msg->r_shoulder_roll(),
        /* r elbow */ msg->r_elbow_yaw(), msg->r_elbow_roll(),
        /* r wrist/hand */ msg->r_wrist_yaw(), msg->r_hand(),
        /* r hip */ msg->r_hip_yaw_pitch(), msg->r_hip_roll(), msg->r_hip_pitch(),
        /* r knee */ msg->r_knee_pitch(),
        /* r ankle */ msg->r_ankle_pitch(), msg->r_ankle_roll(),
        /* speed */ msg->speed());
    }

    __joint_pos_if->msgq_pop();
  }

  // *** Joint stiffness messages
  while (! __joint_stiffness_if->msgq_empty()) {
    if (NaoJointStiffnessInterface::SetStiffnessMessage *msg =
	__joint_stiffness_if->msgq_first_safe(msg))
    {
      /* DCM version, disfunctional due to ALMotion deficiencies
      send_commands(msg->servo(), "Hardness", msg->value(),
		    "Merge", (int)roundf(1000. * msg->time_sec()));
      */

      std::vector<std::string> servos = parse_servo_bitfield(msg->servo());
      std::vector<float> values(servos.size(), msg->value());

      __almotion->post.stiffnessInterpolation(servos, values,
					      msg->time_sec());

    }
    else if (NaoJointStiffnessInterface::SetBodyStiffnessMessage *msg =
	__joint_stiffness_if->msgq_first_safe(msg))
    {
      /* Cannot be used atm because ALMotion will not update its internal
       * belief of stiffness values causing any further motion via DCM
       * or ALMotion to fail
      // use setJointStiffness alias setup in init()
      AL::ALValue cmd;
      cmd.arraySetSize(3);
      cmd[0] = std::string("setJointStiffness");
      cmd[1] = std::string("Merge");
      cmd[2].arraySetSize(1);
      cmd[2][0].arraySetSize(2);
      cmd[2][0][0] = msg->value();
      cmd[2][0][1] = __dcm_time + (int)roundf(1000. * msg->time_sec());
      try {
	__dcm->set(cmd);
      } catch (const AL::ALError &e) {
	logger->log_warn(name(), "Failed to call setJointStiffness: %s",
			 e.toString().c_str());
      }
      */

      __almotion->post.stiffnessInterpolation("Body", msg->value(),
					      msg->time_sec());
    }

    else if (NaoJointStiffnessInterface::SetStiffnessesMessage *msg =
	     __joint_stiffness_if->msgq_first_safe(msg))
    {
      std::vector<float> values(__alljoint_names.getSize());
      values[STIFFJ_HEAD_PITCH] = msg->head_pitch();
      values[STIFFJ_HEAD_YAW] = msg->head_yaw();
      values[STIFFJ_L_SHOULDER_PITCH] = msg->l_shoulder_pitch();
      values[STIFFJ_L_SHOULDER_ROLL] = msg->l_shoulder_roll();
      values[STIFFJ_L_ELBOW_YAW] = msg->l_elbow_yaw();
      values[STIFFJ_L_ELBOW_ROLL] = msg->l_elbow_roll();
      values[STIFFJ_L_HIP_YAW_PITCH] = msg->l_hip_yaw_pitch();
      values[STIFFJ_L_HIP_ROLL] = msg->l_hip_roll();
      values[STIFFJ_L_HIP_PITCH] = msg->l_hip_pitch();
      values[STIFFJ_L_KNEE_PITCH] = msg->l_knee_pitch();
      values[STIFFJ_L_ANKLE_PITCH] = msg->l_ankle_pitch();
      values[STIFFJ_L_ANKLE_ROLL] = msg->l_ankle_roll();

      values[STIFFJ_R_SHOULDER_PITCH] = msg->r_shoulder_pitch();
      values[STIFFJ_R_SHOULDER_ROLL] = msg->r_shoulder_roll();
      values[STIFFJ_R_ELBOW_YAW] = msg->r_elbow_yaw();
      values[STIFFJ_R_ELBOW_ROLL] = msg->r_elbow_roll();
      values[STIFFJ_R_HIP_YAW_PITCH] = msg->r_hip_yaw_pitch();
      values[STIFFJ_R_HIP_ROLL] = msg->r_hip_roll();
      values[STIFFJ_R_HIP_PITCH] = msg->r_hip_pitch();
      values[STIFFJ_R_KNEE_PITCH] = msg->r_knee_pitch();
      values[STIFFJ_R_ANKLE_PITCH] = msg->r_ankle_pitch();
      values[STIFFJ_R_ANKLE_ROLL] = msg->r_ankle_roll();

      if (__robot_type != NaoJointPositionInterface::ROBOTYPE_ROBOCUP) {
	values[STIFFJ_L_WRIST_YAW] = msg->l_wrist_yaw();
	values[STIFFJ_L_HAND] = msg->l_hand();
	values[STIFFJ_R_WRIST_YAW] = msg->r_wrist_yaw();
	values[STIFFJ_R_HAND] = msg->r_hand();
      }

      __almotion->post.stiffnessInterpolation(__alljoint_names, values,
					      msg->time_sec());
    }

    __joint_stiffness_if->msgq_pop();
  }

  // *** Sensor messages
  while (! __sensor_if->msgq_empty()) {
    if (NaoSensorInterface::EmitUltrasonicWaveMessage *msg =
	__sensor_if->msgq_first_safe(msg))
    {
      int value = ultrasonic_value(msg->ultrasonic_direction());
      send_command("US/Actuator/Value", value, "ClearAll", 0);

      __sensor_if->set_ultrasonic_direction(msg->ultrasonic_direction());
      __sensor_highfreq_if->set_ultrasonic_direction(msg->ultrasonic_direction());

    } else if (NaoSensorInterface::StartUltrasonicMessage *msg =
	__sensor_if->msgq_first_safe(msg))
    {
      int value = ultrasonic_value(msg->ultrasonic_direction());
      value += 64;
      send_command("US/Actuator/Value", value, "ClearAll", 0);

      __sensor_if->set_ultrasonic_direction(msg->ultrasonic_direction());
      __sensor_highfreq_if->set_ultrasonic_direction(msg->ultrasonic_direction());

    } else if (NaoSensorInterface::StopUltrasonicMessage *msg =
	__sensor_if->msgq_first_safe(msg))
    {
      send_command("US/Actuator/Value", 0, "ClearAll", 0);

      __sensor_if->set_ultrasonic_direction(NaoSensorInterface::USD_NONE);
      __sensor_highfreq_if->set_ultrasonic_direction(NaoSensorInterface::USD_NONE);
    }

    __sensor_if->msgq_pop();
  }

}


void
NaoQiDCMThread::send_commands(unsigned int servos, std::string what,
			      float value, int time_offset)
{
  /*
  __almotion->wbEnable(false);
  __almotion->wbEnableEffectorControl("LArm", false);
  __almotion->wbEnableEffectorControl("RArm", false);
  __almotion->setWalkArmsEnable(false, false);
  __almotion->killAll();
  */

  std::vector<std::string> servonames = parse_servo_bitfield(servos);

  std::vector<std::string>::iterator s;
  for (s = servonames.begin(); s != servonames.end(); ++s) {
    float v = value;
    if (*s == "HeadYaw") {
      v = CLIP_VALUE(HEAD_YAW, value);
    } else if (*s == "LShoulderPitch") {
      v = CLIP_VALUE(L_SHOULDER_PITCH, value);
    } else if (*s == "RShoulderPitch") {
      v = CLIP_VALUE(R_SHOULDER_PITCH, value);
    }
    send_command(*s + "/" + what + "/Actuator/Value",
		 v, "Merge", time_offset);
  }
}

void
NaoQiDCMThread::send_command(std::string name, float value,
                             std::string kind, int time_offset)
{
  AL::ALValue cmd;

  /*
  printf("Command for %s to %f in %i ms (%i vs. %i)\n",
  	 name.c_str(), value, time_offset, __dcm->getTime(time_offset),
  	 __dcm_time + time_offset);
  */

  cmd.arraySetSize(3);
  cmd[0] = name;
  cmd[1] = kind;
  cmd[2].arraySetSize(1);
  cmd[2][0].arraySetSize(2);
  cmd[2][0][0] = value;
  cmd[2][0][1] = __dcm_time + time_offset;

  __dcm->set(cmd); 
}


std::vector<std::string>
NaoQiDCMThread::parse_servo_bitfield(unsigned int servos)
{
  std::vector<std::string> servonames;

  if ( servos & NaoJointPositionInterface::SERVO_head_yaw )
    servonames.push_back("HeadYaw");

  if ( servos & NaoJointPositionInterface::SERVO_head_pitch )
    servonames.push_back("HeadPitch");

  if ( servos & NaoJointPositionInterface::SERVO_l_shoulder_pitch )
    servonames.push_back("LShoulderPitch");

  if ( servos & NaoJointPositionInterface::SERVO_l_shoulder_roll )
    servonames.push_back("LShoulderRoll");

  if ( servos & NaoJointPositionInterface::SERVO_l_elbow_yaw )
    servonames.push_back("LElbowYaw");

  if ( servos & NaoJointPositionInterface::SERVO_l_elbow_roll )
    servonames.push_back("LElbowRoll");

  if ( servos & NaoJointPositionInterface::SERVO_r_shoulder_pitch )
    servonames.push_back("RShoulderPitch");

  if ( servos & NaoJointPositionInterface::SERVO_r_shoulder_roll )
    servonames.push_back("RShoulderRoll");

  if ( servos & NaoJointPositionInterface::SERVO_r_elbow_yaw )
    servonames.push_back("RElbowYaw");

  if ( servos & NaoJointPositionInterface::SERVO_r_elbow_roll )
    servonames.push_back("RElbowRoll");

  if ( servos & NaoJointPositionInterface::SERVO_l_hip_yaw_pitch )
    servonames.push_back("LHipYawPitch");

  if ( servos & NaoJointPositionInterface::SERVO_l_hip_pitch )
    servonames.push_back("LHipPitch");

  if ( servos & NaoJointPositionInterface::SERVO_l_hip_roll )
    servonames.push_back("LHipRoll");

  if ( servos & NaoJointPositionInterface::SERVO_r_hip_yaw_pitch )
    servonames.push_back("RHipYawPitch");

  if ( servos & NaoJointPositionInterface::SERVO_r_hip_pitch )
    servonames.push_back("RHipPitch");

  if ( servos & NaoJointPositionInterface::SERVO_r_hip_roll )
    servonames.push_back("RHipRoll");

  if ( servos & NaoJointPositionInterface::SERVO_l_knee_pitch )
    servonames.push_back("LKneePitch");

  if ( servos & NaoJointPositionInterface::SERVO_r_knee_pitch )
    servonames.push_back("RKneePitch");

  if ( servos & NaoJointPositionInterface::SERVO_l_ankle_pitch )
    servonames.push_back("LAnklePitch");

  if ( servos & NaoJointPositionInterface::SERVO_l_ankle_roll )
    servonames.push_back("LAnkleRoll");

  if ( servos & NaoJointPositionInterface::SERVO_r_ankle_pitch )
    servonames.push_back("RAnklePitch");

  if ( servos & NaoJointPositionInterface::SERVO_r_ankle_roll )
    servonames.push_back("RAnkleRoll");

  return servonames;
}


int
NaoQiDCMThread::ultrasonic_value(fawkes::NaoSensorInterface::UltrasonicDirection direction)
{
  int value = 0;
  switch (direction) {
  case NaoSensorInterface::USD_LEFT_LEFT:   value =  0; break;
  case NaoSensorInterface::USD_LEFT_RIGHT:  value =  1; break;
  case NaoSensorInterface::USD_RIGHT_LEFT:  value =  2; break;
  case NaoSensorInterface::USD_RIGHT_RIGHT: value =  3; break;
  case NaoSensorInterface::USD_BOTH_BOTH:   value = 12; break;
  default:
    logger->log_warn(name(), "Illegal ultrasonic direction, "
                     "using left-right");
  }
  return value;
}
