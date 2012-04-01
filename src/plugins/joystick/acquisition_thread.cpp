
/***************************************************************************
 *  acqusition_thread.cpp - Thread that retrieves the joystick data
 *
 *  Created: Sat Nov 22 18:14:55 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#include "acquisition_thread.h"
#include "force_feedback.h"

#include <core/threading/mutex.h>
#include <core/exceptions/system.h>

#include <algorithm>
#include <linux/joystick.h>
#include <cstdlib>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cerrno>
#include <cstring>
#include <unistd.h>

using namespace fawkes;


/** @class JoystickAcquisitionThread "acquisition_thread.h"
 * Joystick acqusition thread for Linux joystick API.
 * @see Linux Kernel Documentation (joystick-api.txt)
 * @author Tim Niemueller
 */

/** Constructor. */
JoystickAcquisitionThread::JoystickAcquisitionThread()
  : Thread("JoystickAcquisitionThread", Thread::OPMODE_CONTINUOUS)
{
  set_prepfin_conc_loop(true);
  __data_mutex = NULL;
  __axis_values = NULL;
  __bbhandler = NULL;
  __ff = NULL;
  logger = NULL;
}


/** Alternative constructor.
 * This constructor is meant to be used to create an instance that is used
 * outside of Fawkes.
 * @param device_file joystick device file
 * @param handler BlackBoard handler that will post data to the BlackBoard
 * @param logger logging instance
 */
JoystickAcquisitionThread::JoystickAcquisitionThread(const char *device_file,
						     JoystickBlackBoardHandler *handler,
						     Logger *logger)
  : Thread("JoystickAcquisitionThread", Thread::OPMODE_CONTINUOUS)
{
  set_prepfin_conc_loop(true);
  __data_mutex = NULL;
  __axis_values = NULL;
  __ff = NULL;
  __bbhandler = handler;
  this->logger = logger;
  init(device_file);
}


void
JoystickAcquisitionThread::init()
{
  try {
    __cfg_device_file    = config->get_string("/hardware/joystick/device_file");

  } catch (Exception &e) {
    e.append("Could not read all required config values for %s", name());
    throw;
  }

  init(__cfg_device_file);
}


void
JoystickAcquisitionThread::open_joystick()
{
  __fd = open(__cfg_device_file.c_str(), O_RDONLY);
  if ( __fd == -1 ) {
    throw CouldNotOpenFileException(__cfg_device_file.c_str(), errno,
				    "Opening the joystick device file failed");
  }

  if ( ioctl(__fd, JSIOCGNAME(sizeof(__joystick_name)), __joystick_name) < 0) {
    throw Exception(errno, "Failed to get name of joystick");
  }
  if ( ioctl(__fd, JSIOCGAXES, &__num_axes) < 0 ) {
    throw Exception(errno, "Failed to get number of axes for joystick");
  }
  if ( ioctl(__fd, JSIOCGBUTTONS, &__num_buttons) < 0 ) {
    throw Exception(errno, "Failed to get number of buttons for joystick");
  }

  if (__axis_values == NULL) {
    // memory had not been allocated
    // minimum of 8 because there are 8 axes in the interface
    __axis_array_size = std::max((int)__num_axes, 8);
    __axis_values   = (float *)malloc(sizeof(float) * __axis_array_size);
  } else if ( __num_axes > std::max((int)__axis_array_size, 8) ) {
    // We loose axes as we cannot increase BB interface on-the-fly
    __num_axes = __axis_array_size;
  }

  logger->log_debug(name(), "Joystick device:   %s", __cfg_device_file.c_str());
  logger->log_debug(name(), "Joystick name:     %s", __joystick_name);
  logger->log_debug(name(), "Number of Axes:    %i", __num_axes);
  logger->log_debug(name(), "Number of Buttons: %i", __num_buttons);
  logger->log_debug(name(), "Axis Array Size:   %u", __axis_array_size);

  memset(__axis_values, 0, sizeof(float) * __axis_array_size);
  __pressed_buttons = 0;

  if ( __bbhandler ) {
    __bbhandler->joystick_plugged(__num_axes, __num_buttons);
  }
  __connected = true;
}

void
JoystickAcquisitionThread::open_forcefeedback()
{
  __ff = new JoystickForceFeedback(__joystick_name);
  logger->log_debug(name(), "Force Feedback:    %s", (__ff) ? "Yes" : "No");
  logger->log_debug(name(), "Supported effects:");

  if (__ff->can_rumble())   logger->log_debug(name(), "  rumble");
  if (__ff->can_periodic()) logger->log_debug(name(), "  periodic");
  if (__ff->can_constant()) logger->log_debug(name(), "  constant");
  if (__ff->can_spring())   logger->log_debug(name(), "  spring");
  if (__ff->can_friction()) logger->log_debug(name(), "  friction");
  if (__ff->can_damper())   logger->log_debug(name(), "  damper");
  if (__ff->can_inertia())  logger->log_debug(name(), "  inertia");
  if (__ff->can_ramp())     logger->log_debug(name(), "  ramp");
  if (__ff->can_square())   logger->log_debug(name(), "  square");
  if (__ff->can_triangle()) logger->log_debug(name(), "  triangle");
  if (__ff->can_sine())     logger->log_debug(name(), "  sine");
  if (__ff->can_saw_up())   logger->log_debug(name(), "  saw up");
  if (__ff->can_saw_down()) logger->log_debug(name(), "  saw down");
  if (__ff->can_custom())   logger->log_debug(name(), "  custom");
}

void
JoystickAcquisitionThread::init(std::string device_file)
{
  __new_data = false;
  __cfg_device_file = device_file;
  open_joystick();
  try {
    open_forcefeedback();
  } catch (Exception &e) {
    logger->log_warn(name(), "Initializing force feedback failed, disabling");
    logger->log_warn(name(), e);
  }
  __data_mutex = new Mutex();
}


void
JoystickAcquisitionThread::finalize()
{
  if ( __fd >= 0 )  close(__fd);
  free(__axis_values);
  delete __data_mutex;
}


void
JoystickAcquisitionThread::loop()
{
  if ( __connected ) {
    struct js_event e;

    if ( read(__fd, &e, sizeof(struct js_event)) < (int)sizeof(struct js_event) ) {
      logger->log_warn(name(), "Joystick removed, will try to reconnect.");
      close(__fd);
      __fd = -1;
      __connected = false;
      if ( __bbhandler ) {
	__bbhandler->joystick_unplugged();
      }
      return;
    }

    __data_mutex->lock();
    __new_data = true;

    if ((e.type & ~JS_EVENT_INIT) == JS_EVENT_BUTTON) {
      //logger->log_debug(name(), "Button %u button event: %f", e.number, e.value);
      if (e.number <= 32) {
	if (e.value) {
	  __pressed_buttons |=  (1 << e.number);
	} else {
	  __pressed_buttons &= ~(1 << e.number);
	}
      } else {
	logger->log_warn(name(), "Button value for button > 32, ignoring");
      }
    } else if ((e.type & ~JS_EVENT_INIT) == JS_EVENT_AXIS) {
      if ( e.number >= __axis_array_size ) {
	logger->log_warn(name(),
			 "Got value for axis %u, but only %u axes registered. "
			 "Plugged in a different joystick? Ignoring.",
			 e.number + 1 /* natural numbering */, __axis_array_size);
      } else {
	// Joystick axes usually go positive right, down, twist right, min speed,
	// hat right, and hat down. In the Fawkes coordinate system we actually
	// want opposite directions, hence multiply each value by -1
	__axis_values[e.number] = (e.value == 0) ? 0. : (e.value / -32767.f);
	
	//logger->log_debug(name(), "Axis %u new X: %f",
	//                  axis_index, __axis_values[e.number]);
      }
    }

    __data_mutex->unlock();

    if ( __bbhandler ) {
      __bbhandler->joystick_changed(__pressed_buttons, __axis_values);
    }
  } else {
    // Connection to joystick has been lost
    try {
      open_joystick();
      logger->log_warn(name(), "Joystick plugged in. Delivering data again.");
      try {
	open_forcefeedback();
      } catch (Exception &e) {
	logger->log_warn(name(), "Initializing force feedback failed, disabling");
      }
    } catch (...) {
      // ignored
    }
  }
}


/** Lock data if fresh.
 * If new data has been received since get_distance_data() or get_echo_data()
 * was called last the data is locked, no new data can arrive until you call
 * unlock(), otherwise the lock is immediately released after checking.
 * @return true if the lock was acquired and there is new data, false otherwise
 */
bool
JoystickAcquisitionThread::lock_if_new_data()
{
  __data_mutex->lock();
  if (__new_data) {
    return true;
  } else {
    __data_mutex->unlock();
    return false;
  }
}


/** Unlock data. */
void
JoystickAcquisitionThread::unlock()
{
  __new_data = false;
  __data_mutex->unlock();
}


/** Get number of axes.
 * @return number of axes.
 */
char
JoystickAcquisitionThread::num_axes() const
{
  return __num_axes;
}


/** Get number of buttons.
 * @return number of buttons.
 */
char
JoystickAcquisitionThread::num_buttons() const
{
  return __num_buttons;
}


/** Get joystick name.
 * @return joystick name
 */
const char *
JoystickAcquisitionThread::joystick_name() const
{
  return __joystick_name;
}


/** Pressed buttons.
 * @return bit field where each set bit represents a pressed button.
 */
unsigned int
JoystickAcquisitionThread::pressed_buttons() const
{
  return __pressed_buttons;
}


/** Get values for the axes.
 * @return array of axis values.
 */
float *
JoystickAcquisitionThread::axis_values()
{
  return __axis_values;
}
