
/***************************************************************************
 *  acquisition_thread.h - Thread that retrieves the joystick data
 *
 *  Created: Sat Nov 22 18:10:35 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_JOYSTICK_ACQUISITION_THREAD_H_
#define __PLUGINS_JOYSTICK_ACQUISITION_THREAD_H_

#include "bb_handler.h"

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>

#include <utils/math/types.h>

#include <string>
#include <vector>

namespace fawkes {
  class Mutex;
}

class JoystickForceFeedback;

class JoystickAcquisitionThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect
{
 public:
	JoystickAcquisitionThread();
	JoystickAcquisitionThread(const char *device_file,
                            JoystickBlackBoardHandler *handler,
                            fawkes::Logger *logger);

  virtual void init();
  virtual void finalize();
  virtual void loop();

  bool lock_if_new_data();
  void unlock();

  char               num_axes() const;
  char               num_buttons() const;
  const char *       joystick_name() const;
  unsigned int       pressed_buttons() const;
  float *            axis_values();

  /** Access force feedback of joystick.
   * @return instance of JoystickForceFeedback class for current joystick. */
  JoystickForceFeedback *  ff() const { return ff_; }

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void init(std::string device_file, bool allow_open_fail = false);
  void open_joystick();
  void open_forcefeedback();
  
 private:
  std::string  cfg_device_file_;
  float        cfg_retry_interval_;
  bool         cfg_lazy_init_;
  float        cfg_safety_lockout_timeout_;
  unsigned int cfg_safety_button_mask_;
  unsigned int cfg_safety_bypass_button_mask_;

  bool        safety_combo_[5];
  bool        safety_lockout_;

  int  fd_;
  bool connected_;
  bool just_connected_;
  unsigned int axis_array_size_;
  char num_axes_;
  char num_buttons_;
  char joystick_name_[128];

  bool            new_data_;
  fawkes::Mutex  *data_mutex_;

  unsigned int    pressed_buttons_;
  float          *axis_values_;

  JoystickBlackBoardHandler *bbhandler_;
  JoystickForceFeedback *ff_;
};


#endif
