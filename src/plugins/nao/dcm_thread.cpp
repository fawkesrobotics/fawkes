
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

#include <alproxies/allauncherproxy.h>
#include <alproxies/dcmproxy.h>
#include <alcore/alerror.h>
#include <almemoryfastaccess/almemoryfastaccess.h>

#include <boost/bind.hpp>

#include <interfaces/NaoHardwareInterface.h>

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
		   ACC_X, ACC_Y, ACC_Z, GYR_X, GYR_Y, GYR_REF, ANGLE_X, ANGLE_Y,
		   L_FSR_FL, L_FSR_FR, L_FSR_RL, L_FSR_RR,
		   R_FSR_FL, R_FSR_FR, R_FSR_RL, R_FSR_RR,
		   L_COP_X, L_COP_Y, L_TOTAL_WEIGHT, R_COP_X, R_COP_Y,
		   R_TOTAL_WEIGHT, ULTRASONIC_DISTANCE,
		   L_FOOT_BUMPER_L, L_FOOT_BUMPER_R,
		   R_FOOT_BUMPER_L, R_FOOT_BUMPER_R,
		   HEAD_TOUCH_FRONT, HEAD_TOUCH_MIDDLE, HEAD_TOUCH_REAR,
		   CHEST_BUTTON, BATTERY_CHARGE,
		   SensorTypeN};

#define ACCELEROMETER_G_FACTOR 56.

/** @class NaoQiDCMThread "broker_thread.h"
 * Thread to provide DCM to Fawkes.
 * This thread opens a DCM proxy and updates information about hardware
 * in the blackboard and provides basic setting functionality to move
 * specific servos.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
NaoQiDCMThread::NaoQiDCMThread()
  : Thread("NaoQiDCMThread", Thread::OPMODE_WAITFORWAKEUP)
{
  set_coalesce_wakeups(true);
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
    bool is_dcm_running = launcher->isModulePresent("DCM");

    if (! is_dcm_running) {
      throw Exception("NaoQi DCM is not running");
    }
  } catch (AL::ALError& e) {
    throw Exception("Checking DCM aliveness failed: %s", e.toString().c_str());
  }

  __dcm = naoqi_broker->getDcmProxy();

  __memfa.reset(new AL::ALMemoryFastAccess());

  std::string prefix = "Device/SubDeviceList/";

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
  keys[ULTRASONIC_DISTANCE] = prefix + "US/Sensor/Value";

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

  // Get all values from ALMemory using fastaccess
  __memfa->GetValues(__values);

  __dcm_sigconn =
    __dcm->getGenericProxy()->getModule()->
    atPostProcess(boost::bind(&NaoQiDCMThread::dcm_callback, this));

  __naohw_if = blackboard->open_for_writing<NaoHardwareInterface>("Nao Hardware");
}


void
NaoQiDCMThread::dcm_callback()
{
  //int dcm_time = __dcm->getTime(0);
  // Get all values from ALMemory using fastaccess
  __memfa->GetValues(__values);
  wakeup();
}

void
NaoQiDCMThread::finalize()
{
  __dcm_sigconn.disconnect();

  blackboard->close(__naohw_if);
  __naohw_if = NULL;

  __memfa.reset();
  __dcm.reset();
}


void
NaoQiDCMThread::loop()
{
  /* Head */
  __naohw_if->set_head_yaw(__values[HEAD_YAW]);
  __naohw_if->set_head_pitch(__values[HEAD_PITCH]);
  /* Arms */
  __naohw_if->set_l_shoulder_pitch(__values[L_SHOULDER_PITCH]);
  __naohw_if->set_l_shoulder_roll(__values[L_SHOULDER_ROLL]);
  __naohw_if->set_l_elbow_yaw(__values[L_ELBOW_YAW]);
  __naohw_if->set_l_elbow_roll(__values[L_ELBOW_ROLL]);
  __naohw_if->set_l_wrist_yaw(__values[L_WRIST_YAW]);
  __naohw_if->set_l_hand(__values[L_HAND]);
  __naohw_if->set_r_shoulder_pitch(__values[R_SHOULDER_PITCH]);
  __naohw_if->set_r_shoulder_roll(__values[R_SHOULDER_ROLL]);
  __naohw_if->set_r_elbow_yaw(__values[R_ELBOW_YAW]);
  __naohw_if->set_r_elbow_roll(__values[R_ELBOW_ROLL]);
  __naohw_if->set_r_wrist_yaw(__values[R_WRIST_YAW]);
  __naohw_if->set_r_hand(__values[R_HAND]);
  /* Hip */
  __naohw_if->set_l_hip_yaw_pitch(__values[L_HIP_YAW_PITCH]);
  __naohw_if->set_l_hip_pitch(__values[L_HIP_PITCH]);
  __naohw_if->set_l_hip_roll(__values[L_HIP_ROLL]);
  __naohw_if->set_r_hip_yaw_pitch(__values[R_HIP_YAW_PITCH]);
  __naohw_if->set_r_hip_pitch(__values[R_HIP_PITCH]);
  __naohw_if->set_r_hip_roll(__values[R_HIP_ROLL]);
  /* Knees */
  __naohw_if->set_l_knee_pitch(__values[L_KNEE_PITCH]);
  __naohw_if->set_r_knee_pitch(__values[R_KNEE_PITCH]);
  /* Feet */
  __naohw_if->set_l_ankle_pitch(__values[L_ANKLE_PITCH]);
  __naohw_if->set_l_ankle_roll(__values[L_ANKLE_ROLL]);
  __naohw_if->set_r_ankle_pitch(__values[R_ANKLE_PITCH]);
  __naohw_if->set_r_ankle_roll(__values[R_ANKLE_ROLL]);

  /* FSRs */
  __naohw_if->set_l_fsr_fl(__values[L_FSR_FL]);
  __naohw_if->set_l_fsr_fr(__values[L_FSR_FR]);
  __naohw_if->set_l_fsr_rl(__values[L_FSR_RL]);
  __naohw_if->set_l_fsr_rr(__values[L_FSR_RR]);
  __naohw_if->set_r_fsr_fl(__values[R_FSR_FL]);
  __naohw_if->set_r_fsr_fr(__values[R_FSR_FR]);
  __naohw_if->set_r_fsr_rl(__values[R_FSR_RL]);
  __naohw_if->set_r_fsr_rr(__values[R_FSR_RR]);

  __naohw_if->set_l_cop_x(__values[L_COP_X]);
  __naohw_if->set_l_cop_y(__values[L_COP_Y]);
  __naohw_if->set_l_total_weight(__values[L_TOTAL_WEIGHT]);

  __naohw_if->set_r_cop_x(__values[R_COP_X]);
  __naohw_if->set_r_cop_y(__values[R_COP_Y]);
  __naohw_if->set_r_total_weight(__values[R_TOTAL_WEIGHT]);

  // Buttons and bumpers
  __naohw_if->set_chest_button((__values[CHEST_BUTTON] >= 0.5) ? 1 : 0);
  __naohw_if->set_head_touch_front((__values[HEAD_TOUCH_FRONT] >= 0.5) ? 1 : 0);
  __naohw_if->set_head_touch_middle((__values[HEAD_TOUCH_MIDDLE] >= 0.5) ? 1 : 0);
  __naohw_if->set_head_touch_rear((__values[HEAD_TOUCH_REAR] >= 0.5) ? 1 : 0);

  __naohw_if->set_l_foot_bumper_l((__values[L_FOOT_BUMPER_L] >= 0.5) ? 1 : 0);
  __naohw_if->set_l_foot_bumper_r((__values[L_FOOT_BUMPER_R] >= 0.5) ? 1 : 0);
  __naohw_if->set_r_foot_bumper_l((__values[R_FOOT_BUMPER_L] >= 0.5) ? 1 : 0);
  __naohw_if->set_r_foot_bumper_r((__values[R_FOOT_BUMPER_R] >= 0.5) ? 1 : 0);

  // Inertial measurement unit
  __naohw_if->set_accel_x(__values[ACC_X] / ACCELEROMETER_G_FACTOR);
  __naohw_if->set_accel_y(__values[ACC_Y] / ACCELEROMETER_G_FACTOR);
  __naohw_if->set_accel_z(__values[ACC_Z] / ACCELEROMETER_G_FACTOR);

  __naohw_if->set_gyro_x(__values[GYR_X]);
  __naohw_if->set_gyro_y(__values[GYR_Y]);
  __naohw_if->set_gyro_ref(__values[GYR_REF]);

  __naohw_if->set_angle_x(__values[ANGLE_X]);
  __naohw_if->set_angle_y(__values[ANGLE_Y]);

  // Ultrasonic sound
  __naohw_if->set_ultrasonic_distance(__values[ULTRASONIC_DISTANCE]);

  // Battery
  __naohw_if->set_battery_charge(__values[BATTERY_CHARGE]);

  // Write to blackboard
  __naohw_if->write();
}
