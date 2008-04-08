
/***************************************************************************
 *  motor_thread.h - Motor Thread
 *
 *  Generated: Son Jun 03 00:07:33 2007
 *  Copyright  2007  Martin Liebenberg
 *             2008  Daniel Beck
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __NAVIGATOR_MOTOR_THREAD_H_
#define __NAVIGATOR_MOTOR_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>

class MotorInterface;
class NavigatorThread;
class OmniMotionModel;
class LinearVelocityController;

namespace VMC
{
  class CvmcAPI;
}

class MotorThread
: public Thread,
  public LoggingAspect,
  public BlackBoardAspect,
  public ConfigurableAspect,
  public ClockAspect
{
 public:
  MotorThread();
  
  virtual ~MotorThread();
  
  virtual void loop();
  virtual void init();
  virtual void finalize();
  
 private:
  VMC::CvmcAPI *vmc_api;
  MotorInterface *motor_interface;
  OmniMotionModel* motion_model;
  LinearVelocityController* velocity_controller;

  Time last_loop;
  
  float vx_des;
  float vy_des;
  float omega_des;

  float vx_act;
  float vy_act;
  float omega_act;

  float abs_rot_right;
  float abs_rot_rear;
  float abs_rot_left;

  float odo_delta_x;
  float odo_delta_y;
  float odo_delta_phi;

  bool first_loop;

  // config parameters
  unsigned int loop_time;
  double acceleration_factor;
  double correction_x;
  double correction_y;
  double correction_rotation;
  double correction_translation;
  double gear_reduction;
  double wheel_radius;
  double radius;
  double differential_part;
  double integral_part;
  double linear_part;
  int ticks;
  char* vmc_port;
  bool no_vmc;
};

#endif /* __NAVIGATOR_MOTOR_THREAD_H_ */
