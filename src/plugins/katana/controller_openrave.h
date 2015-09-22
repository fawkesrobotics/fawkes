
/***************************************************************************
 *  controller_openrave.h - OpenRAVE Controller class for katana arm
 *
 *  Created: Sat Jan 07 16:10:54 2012
 *  Copyright  2012-2014  Bahram Maleki-Fard
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

#ifndef __PLUGINS_KATANA_CONTROLLER_OPENRAVE_H_
#define __PLUGINS_KATANA_CONTROLLER_OPENRAVE_H_

#include "controller.h"

#include <core/utils/refptr.h>

#ifdef HAVE_OPENRAVE
#include <plugins/openrave/types.h>
#include <openrave/openrave.h>
#endif

#include <string>
#include <memory>
#include <vector>


namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class OpenRaveConnector;

class KatanaControllerOpenrave : public KatanaController
{
#ifdef HAVE_OPENRAVE
 public:
  KatanaControllerOpenrave(fawkes::OpenRaveConnector* openrave);
  virtual ~KatanaControllerOpenrave();

  // setup
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
  double __x, __y, __z;
  double __phi, __theta, __psi;

  fawkes::OpenRaveConnector* 		__openrave;
  fawkes::OpenRaveEnvironmentPtr	__OR_env;
  fawkes::OpenRaveRobotPtr		__OR_robot;
  fawkes::OpenRaveManipulatorPtr	__OR_manip;
  OpenRAVE::EnvironmentBasePtr 		__env;
  OpenRAVE::RobotBasePtr     		__robot;
  OpenRAVE::RobotBase::ManipulatorPtr   __manip;

  bool __initialized;

  std::vector<short>             __active_motors;

  void update_manipulator();
  void wait_finished();
  void check_init();

  bool motor_oor(unsigned short id);
#endif //HAVE_OPENRAVE
};


} // end of namespace fawkes

#endif
