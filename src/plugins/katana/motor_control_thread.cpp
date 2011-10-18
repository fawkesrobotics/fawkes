
/***************************************************************************
 *  motor_control_thread.cpp - Katana direct motor encoder/value control thread
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

#include "motor_control_thread.h"

#include <cstdlib>
#include <kniBase.h>

/** @class KatanaMotorControlThread "goto_thread.h"
 * Katana motor control thread.
 * This thread moves a single motor to/by a given value in encoders or angle.
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param katana katana linear motion base class
 * @param logger logger
 * @param poll_interval_ms interval in ms between two checks if the
 * final position has been reached
 */
KatanaMotorControlThread::KatanaMotorControlThread(fawkes::RefPtr<CLMBase> katana,
				   fawkes::Logger *logger,
				   unsigned int poll_interval_ms)
  : KatanaMotionThread("KatanaMotorControlThread", katana, logger)
{
  __poll_interval_usec = poll_interval_ms * 1000;
}


/** Set target encoder value
 * @param nr number of motor
 * @param value encoder value
 * @param inc is value incremental? deault:false -> absolute
 */
void
KatanaMotorControlThread::set_encoder(unsigned int nr, int value, bool inc)
{
  __nr	    = nr;
  __encoder = value;

  __is_encoder = true;
  __is_inc = inc;
}

/** Set target angle value
 * @param nr number of motor
 * @param value angle value
 * @param inc is value incremental? deault:false -> absolute
 */
void
KatanaMotorControlThread::set_angle(unsigned int nr, float value, bool inc)
{
  __nr	    = nr;
  __angle = value;

  __is_encoder = false;
  __is_inc = inc;
}


void
KatanaMotorControlThread::once()
{
  unsigned short num_motors = _katana->getNumberOfMotors();
  if(  __nr > num_motors ) {
    _logger->log_warn("KatanaMotorControlThread", "Motor number out of range");
    _finished = true;
    _error_code = fawkes::KatanaInterface::ERROR_UNSPECIFIC;
    return;
  }

  try {
    // non-blocking call to KNI
    if( __is_encoder ) {
      if( __is_inc )
        _katana->moveMotorByEnc(__nr, __encoder);
      else
        _katana->moveMotorToEnc(__nr, __encoder);
    } else {
      if( __is_inc )
        _katana->moveMotorBy(__nr, __angle);
      else
        _katana->moveMotorTo(__nr, __angle);
    }
  } catch (/*KNI*/::Exception &e) {
    _logger->log_warn("KatanaMotorControlThread", "Moving motor %u failed (ignoring): %s", __nr, e.what());
    _finished = true;
    _error_code = fawkes::KatanaInterface::ERROR_CMD_START_FAILED;
    return;
  }

  // TODO: need check for motion end? -> How send 5 commands in a row then?
  // Check for finished motion
  bool final = false;
  const TKatMOT* mot = _katana->GetBase()->GetMOT();
  short num_errors  = 0;

  while (! final) {
    usleep(__poll_interval_usec);
    final = true;
    try {
      mot->arr[__nr].recvPVP(); // get position data for motor
    } catch (/*KNI*/::Exception &e) {
      if (++num_errors <= 10) {
        _logger->log_warn("KatanaMotorControlThread", "Receiving PVP for motor %u failed, retrying", __nr);
        continue;
      } else {
        _logger->log_warn("KatanaMotorControlThread", "Receiving PVP for motor %u failed too often, aborting", __nr);
        _error_code = fawkes::KatanaInterface::ERROR_COMMUNICATION;
        break;
      }
    }


    if (mot->arr[__nr].GetPVP()->msf == MSF_MOTCRASHED) {
      _error_code = fawkes::KatanaInterface::ERROR_MOTOR_CRASHED;
      break;
    }

    final &= std::abs(mot->arr[__nr].GetTPS()->tarpos - mot->arr[__nr].GetPVP()->pos) < 100;
  }

  _logger->log_debug(name(), "Successfully moved motor %u", __nr);

  _finished = true;
}
