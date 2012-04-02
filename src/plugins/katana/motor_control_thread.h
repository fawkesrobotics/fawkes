
/***************************************************************************
 *  motor_control_thread.h - Katana direct motor encoder/value control thread
 *
 *  Created: Sun Mar 13 14:44:24 2011
 *  Copyright  2011  Bahram Maleki-Fard
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

#ifndef __PLUGINS_KATANA_MOTOR_CONTROL_THREAD_H_
#define __PLUGINS_KATANA_MOTOR_CONTROL_THREAD_H_

#include "motion_thread.h"

class KatanaMotorControlThread : public KatanaMotionThread
{
 public:
  KatanaMotorControlThread(fawkes::RefPtr<fawkes::KatanaController> katana, fawkes::Logger *logger,
		   unsigned int poll_interval_ms);

  virtual void set_encoder(unsigned int nr, int value, bool inc=false);
  virtual void set_angle(unsigned int nr, float value, bool inc=false);

  virtual void once();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected:
  virtual void run() { Thread::run(); }

 private:
  unsigned int __nr;
  int __encoder;
  float __angle;

  bool __is_encoder, __is_inc;
  unsigned int __poll_interval_usec;
};


#endif
