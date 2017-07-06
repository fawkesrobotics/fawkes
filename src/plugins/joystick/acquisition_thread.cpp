
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

#include <utils/time/time.h>

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

#define COMBO_IDX_UP      0
#define COMBO_IDX_DOWN    1
#define COMBO_IDX_LEFT    2
#define COMBO_IDX_RIGHT   3
#define COMBO_IDX_RELEASE 4


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
  data_mutex_ = NULL;
  axis_values_ = NULL;
  bbhandler_ = NULL;
  ff_ = NULL;
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
  data_mutex_ = NULL;
  axis_values_ = NULL;
  ff_ = NULL;
  bbhandler_ = handler;
  this->logger = logger;
  safety_lockout_ = true;
  init(device_file);
}


void
JoystickAcquisitionThread::init()
{
  try {
    cfg_device_file_    = config->get_string("/hardware/joystick/device_file");
    cfg_retry_interval_ = config->get_float("/hardware/joystick/retry_interval");
  } catch (Exception &e) {
    e.append("Could not read all required config values for %s", name());
    throw;
  }

  safety_lockout_ = true;
  try {
	  safety_lockout_ = config->get_bool("/hardware/joystick/safety_lockout/enable");
  } catch (Exception &e) {} // ignore, use default
  if (safety_lockout_) {
	  cfg_safety_lockout_timeout_ = config->get_float("/hardware/joystick/safety_lockout/timeout");
	  cfg_safety_button_mask_ = config->get_uint("/hardware/joystick/safety_lockout/button-mask");
	  cfg_safety_bypass_button_mask_ = 0;
	  try {
		  cfg_safety_bypass_button_mask_ = config->get_uint("/hardware/joystick/safety_lockout/bypass-button-mask");
	  } catch (Exception &e) {} // ignore, use default
  }
  for (int i = 0; i < 5; ++i) safety_combo_[i] = false;

  cfg_lazy_init_ = false;
  try {
	  cfg_lazy_init_ = config->get_bool("/hardware/joystick/allow_deferred_initialization");
  } catch (Exception &e) {} // ignore, use default

  try {
	  init(cfg_device_file_, cfg_lazy_init_);
  } catch (Exception &e) {
		if (! cfg_lazy_init_) {
			e.append("Deferred initialization of joystick device disabled");
		}
		throw;
  }

  if (! connected_ && cfg_lazy_init_) {
	  logger->log_info(name(), "Cannot open joystick, deferred initialization enabled");
  }
  
  if (safety_lockout_) {
	  logger->log_info(name(), "To enable joystick, move primary cross all the way in all "
	                   "directions while holding first button. Then let go of button.");
  }
}


void
JoystickAcquisitionThread::open_joystick()
{
  fd_ = open(cfg_device_file_.c_str(), O_RDONLY);
  if ( fd_ == -1 ) {
    throw CouldNotOpenFileException(cfg_device_file_.c_str(), errno,
				    "Opening the joystick device file failed");
  }

  if ( ioctl(fd_, JSIOCGNAME(sizeof(joystick_name_)), joystick_name_) < 0) {
    throw Exception(errno, "Failed to get name of joystick");
  }
  if ( ioctl(fd_, JSIOCGAXES, &num_axes_) < 0 ) {
    throw Exception(errno, "Failed to get number of axes for joystick");
  }
  if ( ioctl(fd_, JSIOCGBUTTONS, &num_buttons_) < 0 ) {
    throw Exception(errno, "Failed to get number of buttons for joystick");
  }

  if (axis_values_ == NULL) {
    // memory had not been allocated
    // minimum of 8 because there are 8 axes in the interface
    axis_array_size_  = std::max((int)num_axes_, 8);
    axis_values_      = (float *)malloc(sizeof(float) * axis_array_size_);
  } else if ( num_axes_ > std::max((int)axis_array_size_, 8) ) {
    // We loose axes as we cannot increase BB interface on-the-fly
    num_axes_ = axis_array_size_;
  }

  logger->log_debug(name(), "Joystick device:   %s", cfg_device_file_.c_str());
  logger->log_debug(name(), "Joystick name:     %s", joystick_name_);
  logger->log_debug(name(), "Number of Axes:    %i", num_axes_);
  logger->log_debug(name(), "Number of Buttons: %i", num_buttons_);
  logger->log_debug(name(), "Axis Array Size:   %u", axis_array_size_);

  memset(axis_values_, 0, sizeof(float) * axis_array_size_);
  pressed_buttons_ = 0;
  
  if ( bbhandler_ ) {
    bbhandler_->joystick_plugged(num_axes_, num_buttons_);
  }
  connected_ = true;
  just_connected_ = true;
}

void
JoystickAcquisitionThread::open_forcefeedback()
{
  ff_ = new JoystickForceFeedback(joystick_name_);
  logger->log_debug(name(), "Force Feedback:    %s", (ff_) ? "Yes" : "No");
  logger->log_debug(name(), "Supported effects:");

  if (ff_->can_rumble())   logger->log_debug(name(), "  rumble");
  if (ff_->can_periodic()) logger->log_debug(name(), "  periodic");
  if (ff_->can_constant()) logger->log_debug(name(), "  constant");
  if (ff_->can_spring())   logger->log_debug(name(), "  spring");
  if (ff_->can_friction()) logger->log_debug(name(), "  friction");
  if (ff_->can_damper())   logger->log_debug(name(), "  damper");
  if (ff_->can_inertia())  logger->log_debug(name(), "  inertia");
  if (ff_->can_ramp())     logger->log_debug(name(), "  ramp");
  if (ff_->can_square())   logger->log_debug(name(), "  square");
  if (ff_->can_triangle()) logger->log_debug(name(), "  triangle");
  if (ff_->can_sine())     logger->log_debug(name(), "  sine");
  if (ff_->can_saw_up())   logger->log_debug(name(), "  saw up");
  if (ff_->can_saw_down()) logger->log_debug(name(), "  saw down");
  if (ff_->can_custom())   logger->log_debug(name(), "  custom");
}

void
JoystickAcquisitionThread::init(std::string device_file, bool allow_open_fail)
{
	fd_ = -1;
	connected_ = false;
	just_connected_ = false;
	new_data_ = false;

  cfg_device_file_ = device_file;
  try {
	  open_joystick();
	  try {
		  open_forcefeedback();
	  } catch (Exception &e) {
		  logger->log_warn(name(), "Initializing force feedback failed, disabling");
		  logger->log_warn(name(), e);
	  }
  } catch (Exception &e) {
	  if (! allow_open_fail) throw;
  }
  data_mutex_ = new Mutex();
}


void
JoystickAcquisitionThread::finalize()
{
  if ( fd_ >= 0 )  close(fd_);
  if (axis_values_)  free(axis_values_);
  delete data_mutex_;
}


void
JoystickAcquisitionThread::loop()
{
  if ( connected_ ) {
    struct js_event e;

    long int timeout_sec  = (long int)truncf(cfg_safety_lockout_timeout_);
    long int timeout_usec = (cfg_safety_lockout_timeout_ - timeout_sec) * 10000000;
    timeval timeout = {timeout_sec, timeout_usec};

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);

    int rv = 0;
    rv = select(fd_ + 1, &read_fds, NULL, NULL, &timeout);

    if (rv == 0) {
	    if (! safety_lockout_) {
		    logger->log_warn(name(), "No action for %.2f seconds, re-enabling safety lockout",
		                     cfg_safety_lockout_timeout_);
		    safety_lockout_ = true;
		    for (int i = 0; i < 5; ++i) safety_combo_[i] = false;
	    }
	    new_data_ = false;
	    return;
    }
    
    if (rv == -1 || read(fd_, &e, sizeof(struct js_event)) < (int)sizeof(struct js_event)) {
	    logger->log_warn(name(), "Joystick removed, will try to reconnect.");
	    close(fd_);
	    fd_ = -1;
	    connected_ = false;
	    just_connected_ = false;
	    safety_lockout_ = true;
	    new_data_ = false;
	    if ( bbhandler_ ) {
		    bbhandler_->joystick_unplugged();
	    }
	    return;
    }

    data_mutex_->lock();

    new_data_ = ! safety_lockout_;
    unsigned int last_pressed_buttons = pressed_buttons_;
    
    if ((e.type & ~JS_EVENT_INIT) == JS_EVENT_BUTTON) {
      //logger->log_debug(name(), "Button %u button event: %f", e.number, e.value);
      if (e.number <= 32) {
	      if (e.value) {
		      pressed_buttons_ |=  (1 << e.number);
	      } else {
		      pressed_buttons_ &= ~(1 << e.number);
	      }
      } else {
	      logger->log_warn(name(), "Button value for button > 32, ignoring");
      }
    } else if ((e.type & ~JS_EVENT_INIT) == JS_EVENT_AXIS) {
	    if ( e.number >= axis_array_size_ ) {
		    logger->log_warn(name(),
		                     "Got value for axis %u, but only %u axes registered. "
		                     "Plugged in a different joystick? Ignoring.",
		                     e.number + 1 /* natural numbering */, axis_array_size_);
	    } else {
		    // Joystick axes usually go positive right, down, twist right, min speed,
		    // hat right, and hat down. In the Fawkes coordinate system we actually
		    // want opposite directions, hence multiply each value by -1
		    axis_values_[e.number] = (e.value == 0) ? 0. : (e.value / -32767.f);
	
		    //logger->log_debug(name(), "Axis %u new X: %f",
		    //                  axis_index, axis_values_[e.number]);
	    }
    }

    // As a special case, allow a specific button combination to be
    // written even during safety lockout. Can be used to implement
    // an emergency stop, for example.
    if (safety_lockout_ &&
        ((cfg_safety_bypass_button_mask_ & pressed_buttons_) ||
         ((cfg_safety_bypass_button_mask_ & last_pressed_buttons) && pressed_buttons_ == 0)))
    {
	    new_data_ = true;
    }
    
    data_mutex_->unlock();

    if (safety_lockout_) {
	    // the actual axis directions don't matter, we are just interested
	    // that they take both extremes once.
	    if (num_axes_ < 2 || num_buttons_ == 0) {
		    safety_combo_[COMBO_IDX_UP]      = true;
		    safety_combo_[COMBO_IDX_DOWN]    = true;
		    safety_combo_[COMBO_IDX_RIGHT]   = true;
		    safety_combo_[COMBO_IDX_LEFT]    = true;
		    safety_combo_[COMBO_IDX_RELEASE] = true;
	    } else {
		    if (pressed_buttons_ & cfg_safety_button_mask_) {
			    if (axis_values_[0] >  0.9)  safety_combo_[COMBO_IDX_UP]    = true;
			    if (axis_values_[0] < -0.9)  safety_combo_[COMBO_IDX_DOWN]  = true;
			    if (axis_values_[1] >  0.9)  safety_combo_[COMBO_IDX_RIGHT] = true;
			    if (axis_values_[1] < -0.9)  safety_combo_[COMBO_IDX_LEFT]  = true;
		    }
		    if (safety_combo_[COMBO_IDX_UP] && safety_combo_[COMBO_IDX_DOWN] &&
		        safety_combo_[COMBO_IDX_LEFT] && safety_combo_[COMBO_IDX_RIGHT] &&
		        pressed_buttons_ == 0) {
			    safety_combo_[COMBO_IDX_RELEASE] = true;
		    }
	    }

	    if (safety_combo_[COMBO_IDX_UP] && safety_combo_[COMBO_IDX_DOWN] &&
	        safety_combo_[COMBO_IDX_LEFT] && safety_combo_[COMBO_IDX_RIGHT] &&
	        safety_combo_[COMBO_IDX_RELEASE])
	    {
		    logger->log_warn(name(), "Joystick safety lockout DISABLED (combo received)");
		    safety_lockout_ = false;
	    }
    } else {
	    if ( bbhandler_ ) {
		    bbhandler_->joystick_changed(pressed_buttons_, axis_values_);
	    }
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
		} catch (Exception &e) {
			Time duration(cfg_retry_interval_);
			duration.wait_systime();
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
  data_mutex_->lock();
  if (new_data_ || just_connected_) {
	  just_connected_ = false;
    return true;
  } else {
    data_mutex_->unlock();
    return false;
  }
}


/** Unlock data. */
void
JoystickAcquisitionThread::unlock()
{
  new_data_ = false;
  data_mutex_->unlock();
}


/** Get number of axes.
 * @return number of axes.
 */
char
JoystickAcquisitionThread::num_axes() const
{
  return num_axes_;
}


/** Get number of buttons.
 * @return number of buttons.
 */
char
JoystickAcquisitionThread::num_buttons() const
{
  return num_buttons_;
}


/** Get joystick name.
 * @return joystick name
 */
const char *
JoystickAcquisitionThread::joystick_name() const
{
  return joystick_name_;
}


/** Pressed buttons.
 * @return bit field where each set bit represents a pressed button.
 */
unsigned int
JoystickAcquisitionThread::pressed_buttons() const
{
	if (! safety_lockout_) {
		return pressed_buttons_;
	} else if (pressed_buttons_ & cfg_safety_bypass_button_mask_) {
		return pressed_buttons_ & cfg_safety_bypass_button_mask_;
	} else {
		return 0;
	}
}


/** Get values for the axes.
 * @return array of axis values.
 */
float *
JoystickAcquisitionThread::axis_values()
{
	if (safety_lockout_) {
		memset(axis_values_, 0, axis_array_size_ * sizeof(float));
	}
  return axis_values_;
}
