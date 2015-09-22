
/***************************************************************************
 *  goto_thread.cpp - Katana goto one-time thread
 *
 *  Created: Wed Jun 10 11:45:31 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *             2011-2014  Bahram Maleki-Fard
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

#include "goto_thread.h"
#include "controller.h"
#include "exception.h"

#include <interfaces/KatanaInterface.h>

#include <cstdlib>
#include <unistd.h>

/** @class KatanaGotoThread "goto_thread.h"
 * Katana linear goto thread.
 * This thread moves the arm into a specified position.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param katana katana controller base class
 * @param logger logger
 * @param poll_interval_ms interval in ms between two checks if the
 * final position has been reached
 */
KatanaGotoThread::KatanaGotoThread(fawkes::RefPtr<fawkes::KatanaController> katana,
				   fawkes::Logger *logger,
				   unsigned int poll_interval_ms)
  : KatanaMotionThread("KatanaGotoThread", katana, logger)
{
  __poll_interval_usec = poll_interval_ms * 1000;
}


/** Set target position.
 * @param x X coordinate relative to base
 * @param y Y coordinate relative to base
 * @param z Z coordinate relative to base
 * @param phi Phi Euler angle of tool
 * @param theta Theta Euler angle of tool
 * @param psi Psi Euler angle of tool
 */
void
KatanaGotoThread::set_target(float x, float y, float z,
			     float phi, float theta, float psi)
{
  __x     = x;
  __y     = y;
  __z     = z;
  __phi   = phi;
  __theta = theta;
  __psi   = psi;
}

void
KatanaGotoThread::once()
{
  try {
    // non-blocking call
    _katana->move_to(__x, __y, __z, __phi, __theta, __psi);
  } catch (fawkes::KatanaNoSolutionException &e) {
    _logger->log_warn("KatanaGotoThread", "Initiating goto failed (no solution, ignoring): %s", e.what());
    _finished = true;
    _error_code = fawkes::KatanaInterface::ERROR_NO_SOLUTION;
    return;
  } catch (fawkes::Exception &e) {
    _logger->log_warn("KatanaGotoThread", "Initiating goto failed (ignoring): %s", e.what());
    _finished = true;
    _error_code = fawkes::KatanaInterface::ERROR_CMD_START_FAILED;
    return;
  }

 // check if final
  bool final = false;
  short num_errors  = 0;
  while ( !final ) {
    usleep(__poll_interval_usec);
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

  _logger->log_debug(name(), "Position (%f,%f,%f, %f,%f,%f) reached",
		       __x, __y, __z, __phi, __theta, __psi);

  _finished = true;
}
