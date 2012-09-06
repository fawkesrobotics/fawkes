
/***************************************************************************
 *  controller_kni.cpp - KNI Controller class for katana arm
 *
 *  Created: Tue Jan 03 11:40:31 2012
 *  Copyright  2012  Bahram Maleki-Fard, AllemaniACs RoboCup Team
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

#include "controller_kni.h"
#include "exception.h"

#include <core/exceptions/software.h>

#include <kniBase.h>
#include <common/MathHelperFunctions.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class KatanaControllerKni <plugins/katana/controller_kni.h>
 * Controller class for a Neuronics Katana, using libkni to interact
 * with the real Katana arm.
 * @author Bahram Maleki-Fard
 */

/** Constructor. */
KatanaControllerKni::KatanaControllerKni()
{
  // setting default setup values
  __cfg_device = "/dev/ttyS0";
  __cfg_kni_conffile = "/etc/kni3/hd300/katana6M180.cfg";
  __cfg_read_timeout = 100;
  __cfg_write_timeout = 0;

  __gripper_last_pos.clear();
  __gripper_last_pos.resize(2);
}

/** Destructor. */
KatanaControllerKni::~KatanaControllerKni()
{
  // Setting to NULL also deletes instance (RefPtr)
  __katana = NULL;

  __device.reset();
  __protocol.reset();
}

/** Setup parameters needed to initialize Katana arm with libkni.
 * @param device device string, e.g. "/dev/ttyS0"
 * @param kni_conffile path to kni configfile, e.g. "/etc/kni3/hd300/katana6M180.cfg"
 * @param read_timeout timeout for read operations, in ms
 * @param write_timeout timeout for write operations, in ms
 */
void
KatanaControllerKni::setup(std::string& device, std::string& kni_conffile,
			   unsigned int read_timeout, unsigned int write_timeout)
{
  __cfg_device = device;
  __cfg_kni_conffile = kni_conffile;
  __cfg_read_timeout = read_timeout;
  __cfg_write_timeout = write_timeout;
}

void
KatanaControllerKni::init()
{
  try {
    TCdlCOMDesc ccd = {0, 57600, 8, 'N', 1, (int)__cfg_read_timeout, (int)__cfg_write_timeout};
    __device.reset(new CCdlCOM(ccd, __cfg_device.c_str()));

    __protocol.reset(new CCplSerialCRC());
    __protocol->init(__device.get());

    __katana = RefPtr<CLMBase>(new CLMBase());
    __katana->create(__cfg_kni_conffile.c_str(), __protocol.get());
    __katbase = __katana->GetBase();
    __sensor_ctrl = &__katbase->GetSCT()->arr[0];

    __katbase->recvECH();
  } catch (/*KNI*/::Exception &e) {
    throw fawkes::Exception("KNI Exception:%s", e.what());
  }

  try {
    __motor_init.resize(__katana->getNumberOfMotors());
    for(unsigned int i=0; i<__motor_init.size(); i++) {
      __motor_init.at(i) = *(__katbase->GetMOT()->arr[i].GetInitialParameters());
    }
  } catch (/*KNI*/::Exception &e) {
    throw fawkes::Exception("KNI Exception:%s", e.what());
  }
}

void
KatanaControllerKni::set_max_velocity(unsigned int vel)
{
  try {
    __katana->setRobotVelocityLimit((int)vel);
  } catch (/*KNI*/::Exception &e) {
    throw fawkes::Exception("KNI Exception:%s", e.what());
  }
}


bool
KatanaControllerKni::final()
{
  if( __active_motors.size() < 1 )
    return true;

  bool final = true;
  for(unsigned int i=0; i<__active_motors.size(); i++) {
    final &= motor_final(__active_motors.at(i));
  }
  cleanup_active_motors();
  return final;
}

bool
KatanaControllerKni::joint_angles()
{
  return true;
}
bool
KatanaControllerKni::joint_encoders()
{
  return true;
}

void
KatanaControllerKni::calibrate()
{
  try {
    __katana->calibrate();
  } catch (/*KNI*/::Exception &e) {
    throw fawkes::Exception("KNI Exception:%s", e.what());
  }
}

void
KatanaControllerKni::stop()
{
  try {
    __katana->freezeRobot();
  } catch (/*KNI*/::Exception &e) {
    throw fawkes::Exception("KNI Exception:%s", e.what());
  }
}

void
KatanaControllerKni::turn_on()
{
  try {
    __katana->switchRobotOn();
  } catch (/*KNI*/::Exception &e) {
    throw fawkes::Exception("KNI Exception:%s", e.what());
  }
}

void
KatanaControllerKni::turn_off()
{
  try {
    __katana->switchRobotOff();
  } catch (/*KNI*/::Exception &e) {
    throw fawkes::Exception("KNI Exception:%s", e.what());
  }
}

void
KatanaControllerKni::read_coordinates(bool refresh)
{
  try {
    __katana->getCoordinates(__x, __y, __z, __phi, __theta, __psi, refresh);
  } catch (/*KNI*/::Exception &e) {
    throw fawkes::Exception("KNI Exception:%s", e.what());
  }
}

void
KatanaControllerKni::read_motor_data()
{
  try {
    if( __active_motors.size() == (unsigned short)__katana->getNumberOfMotors() ) {
      __katbase->recvMPS(); // get position for all motors
      __katbase->recvGMS(); // get status flags for all motors
    } else {
      const TKatMOT* mot = __katbase->GetMOT();
      for(unsigned int i=0; i<__active_motors.size(); i++) {
        mot->arr[__active_motors.at(i)].recvPVP(); // get position data for motor
      }
    }
  } catch (/*KNI*/::Exception &e) {
    throw fawkes::Exception("KNI Exception:%s", e.what());
  }
}

void
KatanaControllerKni::read_sensor_data()
{
  try {
    __sensor_ctrl->recvDAT();
  } catch (/*KNI*/::ParameterReadingException &e) {
    throw fawkes::Exception("KNI ParameterReadingException:%s", e.what());
  } catch (/*KNI*/::Exception &e) {
    throw fawkes::Exception("KNI Exception:%s", e.what());
  }
}

void
KatanaControllerKni::gripper_open(bool blocking)
{
  try {
    __katana->openGripper(blocking);
  } catch (/*KNI*/::Exception &e) {
    throw fawkes::Exception("KNI Exception:%s", e.what());
  }

  __active_motors.clear();
  __active_motors.resize(1);
  __active_motors[0] = __katbase->GetMOT()->cnt - 1;

  __gripper_last_pos.clear();
  __gripper_last_pos[0] = __katbase->GetMOT()->arr[__active_motors[0]].GetPVP()->pos;
  __gripper_last_pos[1] = 0; //counter
}

void
KatanaControllerKni::gripper_close(bool blocking)
{
  try {
    __katana->closeGripper(blocking);
  } catch (/*KNI*/::Exception &e) {
    throw fawkes::Exception("KNI Exception:%s", e.what());
  }

  __active_motors.clear();
  __active_motors.resize(1);
  __active_motors[0] = __katbase->GetMOT()->cnt - 1;

  __gripper_last_pos.clear();
  __gripper_last_pos[0] = __katbase->GetMOT()->arr[__active_motors[0]].GetPVP()->pos;
  __gripper_last_pos[1] = 0; //counter
}

void
KatanaControllerKni::move_to(float x, float y, float z, float phi, float theta, float psi, bool blocking)
{
  cleanup_active_motors();

  try {
    __katana->moveRobotTo(__x, __y, __z, __phi, __theta, __psi, blocking);
  } catch (KNI::NoSolutionException &e) {
    throw fawkes::KatanaNoSolutionException("KNI NoSolutionException:%s", e.what());
  } catch (/*KNI*/::Exception &e) {
    throw fawkes::Exception("KNI Exception:%s", e.what());
    return;
  }

  for(short i=0; i<__katana->getNumberOfMotors(); ++i) {
    add_active_motor(i);
  }
}

void
KatanaControllerKni::move_to(std::vector<int> encoders, bool blocking)
{
  cleanup_active_motors();

  try {
    __katana->moveRobotToEnc(encoders);
  } catch (/*KNI*/::Exception &e) {
    throw fawkes::Exception("KNI Exception:%s", e.what());
  }

  for(unsigned short i=0; i<encoders.size(); ++i) {
    add_active_motor(i);
  }
}

void
KatanaControllerKni::move_to(std::vector<float> angles, bool blocking)
{
  std::vector<int> encoders;

  try {
    for(unsigned int i=0; i<angles.size(); i++) {
      encoders.push_back(KNI_MHF::rad2enc( (double)angles.at(i), __motor_init.at(i).angleOffset, __motor_init.at(i).encodersPerCycle, __motor_init.at(i).encoderOffset, __motor_init.at(i).rotationDirection));
    }
  } catch (/*KNI*/::Exception &e) {
    throw fawkes::Exception("KNI Exception:%s", e.what());
  }

  move_to(encoders, blocking);
}

void
KatanaControllerKni::move_motor_to(unsigned short id, int enc, bool blocking)
{
  if( motor_oor(id) )
    throw fawkes::KatanaOutOfRangeException("Motor out of range.");

  cleanup_active_motors();

  try {
    __katana->moveMotorToEnc(id, enc);
  } catch (/*KNI*/::Exception &e) {
    throw fawkes::Exception("KNI Exception:%s", e.what());
  }

  add_active_motor(id);
}

void
KatanaControllerKni::move_motor_to(unsigned short id, float angle, bool blocking)
{
  if( motor_oor(id) )
    throw fawkes::KatanaOutOfRangeException("Motor out of range.");

  cleanup_active_motors();

  try {
    __katana->moveMotorTo(id, angle);
  } catch (/*KNI*/::Exception &e) {
    throw fawkes::Exception("KNI Exception:%s", e.what());
  }

  add_active_motor(id);
}

void
KatanaControllerKni::move_motor_by(unsigned short id, int enc, bool blocking)
{
  if( motor_oor(id) )
    throw fawkes::KatanaOutOfRangeException("Motor out of range.");

  cleanup_active_motors();

  try {
    __katana->moveMotorByEnc(id, enc);
  } catch (/*KNI*/::Exception &e) {
    throw fawkes::Exception("KNI Exception:%s", e.what());
  }

  add_active_motor(id);
}

void
KatanaControllerKni::move_motor_by(unsigned short id, float angle, bool blocking)
{
  if( motor_oor(id) )
    throw fawkes::KatanaOutOfRangeException("Motor out of range.");

  cleanup_active_motors();

  try {
    __katana->moveMotorBy(id, angle);
  } catch (/*KNI*/::Exception &e) {
    throw fawkes::Exception("KNI Exception:%s", e.what());
  }

  add_active_motor(id);
}


// getters
double
KatanaControllerKni::x()
{
  return __x;
}

double
KatanaControllerKni::y()
{
  return __y;
}

double
KatanaControllerKni::z()
{
  return __z;
}

double
KatanaControllerKni::phi()
{
  return __phi;
}

double
KatanaControllerKni::theta()
{
  return __theta;
}

double
KatanaControllerKni::psi()
{
  return __psi;
}

void
KatanaControllerKni::get_sensors(std::vector<int>& to, bool refresh)
{
  if( refresh )
    read_sensor_data();

  try {
    const TSctDAT *sensor_data = __sensor_ctrl->GetDAT();

    const int num_sensors = (size_t)sensor_data->cnt;
    to.clear();
    to.resize(num_sensors);

    for (int i = 0; i < num_sensors; ++i) {
      to[i] = sensor_data->arr[i];
    }
  } catch (/*KNI*/::Exception &e) {
    throw fawkes::Exception("KNI Exception:%s", e.what());
  }
}

void
KatanaControllerKni::get_encoders(std::vector<int>& to, bool refresh)
{
  try {
    std::vector<int> encoders = __katana->getRobotEncoders(refresh);

    to.clear();
    to = encoders;
  } catch (/*KNI*/::Exception &e) {
    throw fawkes::Exception("KNI Exception:%s", e.what());
  }
}

void
KatanaControllerKni::get_angles(std::vector<float>& to, bool refresh)
{
  try {
    std::vector<int> encoders = __katana->getRobotEncoders(refresh);

    to.clear();
    for(unsigned int i=0; i<encoders.size(); i++) {
      to.push_back(KNI_MHF::enc2rad( encoders.at(i), __motor_init.at(i).angleOffset, __motor_init.at(i).encodersPerCycle, __motor_init.at(i).encoderOffset, __motor_init.at(i).rotationDirection));
    }
  } catch (/*KNI*/::Exception &e) {
    throw fawkes::Exception("KNI Exception:%s", e.what());
  }
}


bool
KatanaControllerKni::motor_oor(unsigned short id)
{
  return id > (unsigned short)__katana->getNumberOfMotors();
}

bool
KatanaControllerKni::motor_final(unsigned short id)
{
  CMotBase mot = __katbase->GetMOT()->arr[id];
  if (mot.GetPVP()->msf == MSF_MOTCRASHED)
    throw fawkes::KatanaMotorCrashedException("Motor %u crashed.", id);

  // extra check for gripper, consider final if not moved for a while
  unsigned short gripper_not_moved = 0;
  if (id == __katbase->GetMOT()->cnt - 1) {
    if (__gripper_last_pos[0] == mot.GetPVP()->pos) {
      __gripper_last_pos[1] += 1;
    } else {
      __gripper_last_pos[0] = mot.GetPVP()->pos;
      __gripper_last_pos[1] = 0;
    }
    gripper_not_moved = __gripper_last_pos[1];
  }

  return (std::abs(mot.GetTPS()->tarpos - mot.GetPVP()->pos) < 10)
      or (gripper_not_moved > 3);
}

void
KatanaControllerKni::cleanup_active_motors()
{
  for(unsigned int i=0; i<__active_motors.size(); ++i) {
    if( motor_final(__active_motors.at(i)) ) {
      __active_motors.erase(__active_motors.begin()+i);
      --i;
    }
  }
}

void
KatanaControllerKni::add_active_motor(unsigned short id)
{
  for(unsigned int i=0; i<__active_motors.size(); ++i) {
    if( __active_motors.at(i) == id ) {
      return;
    }
  }
  __active_motors.push_back(id);
}

} // end of namespace fawkes
