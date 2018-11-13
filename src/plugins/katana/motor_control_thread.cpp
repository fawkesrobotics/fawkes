
/***************************************************************************
 *  motor_control_thread.cpp - Katana direct motor encoder/value control thread
 *
 *  Created: Sun Mar 13 14:44:24 2011
 *  Copyright  2011-2014  Bahram Maleki-Fard
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

#include "motor_control_thread.h"
#include "controller.h"
#include "exception.h"

#include <interfaces/KatanaInterface.h>

#include <cstdlib>
#include <unistd.h>

/** @class KatanaMotorControlThread "goto_thread.h"
 * Katana motor control thread.
 * This thread moves a single motor to/by a given value in encoders or angle.
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param katana katana controller base class
 * @param logger logger
 * @param poll_interval_ms interval in ms between two checks if the
 * final position has been reached
 */
KatanaMotorControlThread::KatanaMotorControlThread(fawkes::RefPtr<fawkes::KatanaController> katana,
				   fawkes::Logger *logger,
				   unsigned int poll_interval_ms)
  : KatanaMotionThread("KatanaMotorControlThread", katana, logger)
{
  poll_interval_usec_ = poll_interval_ms * 1000;
}


/** Set target encoder value
 * @param nr number of motor
 * @param value encoder value
 * @param inc is value incremental? deault:false -> absolute
 */
void
KatanaMotorControlThread::set_encoder(unsigned int nr, int value, bool inc)
{
  nr_	    = nr;
  encoder_ = value;

  is_encoder_ = true;
  is_inc_ = inc;
}

/** Set target angle value
 * @param nr number of motor
 * @param value angle value
 * @param inc is value incremental? deault:false -> absolute
 */
void
KatanaMotorControlThread::set_angle(unsigned int nr, float value, bool inc)
{
  nr_	    = nr;
  angle_ = value;

  is_encoder_ = false;
  is_inc_ = inc;
}


void
KatanaMotorControlThread::once()
{
  try {
    // non-blocking call to KNI
    if( is_encoder_ ) {
      if( is_inc_ )

        _katana->move_motor_by(nr_, encoder_);
      else
        _katana->move_motor_to(nr_, encoder_);
    } else {
      if( is_inc_ )
        _katana->move_motor_by(nr_, angle_);
      else
        _katana->move_motor_to(nr_, angle_);
    }
  } catch (fawkes::KatanaOutOfRangeException &e) {
    _logger->log_warn("KatanaMotorControlThread", "Motor %u out of range. Ex%s", nr_, e.what());
    _finished = true;
    _error_code = fawkes::KatanaInterface::ERROR_UNSPECIFIC;
    return;
  } catch (fawkes::Exception &e) {
    _logger->log_warn("KatanaMotorControlThread", "Moving motor %u failed (ignoring): %s", nr_, e.what());
    _finished = true;
    _error_code = fawkes::KatanaInterface::ERROR_CMD_START_FAILED;
    return;
  }

  // check if final
  bool final = false;
  short num_errors  = 0;
  while ( !final ) {
    usleep(poll_interval_usec_);
    try {
      _katana->read_sensor_data();
      _katana->read_motor_data();
    } catch (fawkes::Exception &e) {
      if (++num_errors <= 10) {
        _logger->log_warn("KatanaMotorControlThread", "Reading sensor/motor data failed, retrying");
        continue;
      } else {
        _logger->log_warn("KatanaMotorControlThread", "Receiving sensor/motor data failed too often, aborting");
        _error_code = fawkes::KatanaInterface::ERROR_COMMUNICATION;
        break;
      }
    }

    try {
      final = _katana->final();
    } catch (fawkes::KatanaMotorCrashedException &e) {
      _logger->log_warn("KatanaMotorControlTrhead", e.what());
      _error_code = fawkes::KatanaInterface::ERROR_MOTOR_CRASHED;
      break;
    }
  }

  _logger->log_debug(name(), "Successfully moved motor %u", nr_);

  _finished = true;
}
