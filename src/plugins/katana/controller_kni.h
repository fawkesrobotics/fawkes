
/***************************************************************************
 *  controller_kni.h - KNI Controller class for katana arm
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

#ifndef _PLUGINS_KATANA_CONTROLLER_KNI_H_
#define _PLUGINS_KATANA_CONTROLLER_KNI_H_

#include "controller.h"

#include <core/utils/refptr.h>
#include <string>
#include <memory>
#include <vector>

// Classes from libkni (KNI)
class CCdlCOM;
class CCplSerialCRC;
class CLMBase;
class CKatBase;
class CSctBase;
struct TMotInit;

namespace fawkes {

//class RefPtr;

class KatanaControllerKni : public KatanaController
{
 public:
  KatanaControllerKni();
  virtual ~KatanaControllerKni();

  // setup
  virtual void setup(std::string& device, std::string& kni_conffile,
                     unsigned int read_timeout, unsigned int write_timeout);
  virtual void init();
  virtual void set_max_velocity(unsigned int vel);


  // status checking
  virtual bool final();
  virtual bool joint_angles();
  virtual bool joint_encoders();

  // commands
  virtual void calibrate();
  virtual void stop();
  virtual void turn_on();
  virtual void turn_off();
  virtual void read_coordinates(bool refresh = false);
  virtual void read_motor_data();
  virtual void read_sensor_data();
  virtual void gripper_open(bool blocking = false);
  virtual void gripper_close(bool blocking = false);
  virtual void move_to(float x, float y, float z, float phi, float theta, float psi, bool blocking = false);
  virtual void move_to(std::vector<int> encoders, bool blocking = false);
  virtual void move_to(std::vector<float> angles, bool blocking = false);
  virtual void move_motor_to(unsigned short id, int enc, bool blocking = false);
  virtual void move_motor_to(unsigned short id, float angle, bool blocking = false);
  virtual void move_motor_by(unsigned short id, int enc, bool blocking = false);
  virtual void move_motor_by(unsigned short id, float angle, bool blocking = false);

  // getters
  virtual double x();
  virtual double y();
  virtual double z();
  virtual double phi();
  virtual double theta();
  virtual double psi();
  virtual void get_sensors(std::vector<int>& to, bool refresh = false);
  virtual void get_encoders(std::vector<int>& to, bool refresh = false);
  virtual void get_angles(std::vector<float>& to, bool refresh = false);

 private:
  double x_, y_, z_;
  double phi_, theta_, psi_;

  std::string    cfg_device_;
  std::string    cfg_kni_conffile_;
  unsigned int   cfg_read_timeout_;
  unsigned int   cfg_write_timeout_;

  fawkes::RefPtr<CLMBase>        katana_;
#if __cplusplus >= 201103L
  std::unique_ptr<CCdlCOM>       device_;
  std::unique_ptr<CCplSerialCRC> protocol_;
#else
  std::auto_ptr<CCdlCOM>         device_;
  std::auto_ptr<CCplSerialCRC>   protocol_;
#endif
  CKatBase                      *katbase_;
  CSctBase                      *sensor_ctrl_;
  std::vector<TMotInit>          motor_init_;

  std::vector<short>             active_motors_;
  std::vector<int>               gripper_last_pos_;

  bool motor_oor(unsigned short id);
  bool motor_final(unsigned short id);
  void cleanup_active_motors();
  void add_active_motor(unsigned short id);

};


} // end of namespace fawkes

#endif
