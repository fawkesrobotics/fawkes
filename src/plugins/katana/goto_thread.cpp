
/***************************************************************************
 *  goto_thread.cpp - Katana goto one-time thread
 *
 *  Created: Wed Jun 10 11:45:31 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#include <cstdlib>
#include <kniBase.h>

/** @class KatanaGotoThread "goto_thread.h"
 * Katana linear goto thread.
 * This thread moves the arm into a specified position.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param katana katana linear motion base class
 * @param logger logger
 * @param poll_interval_ms interval in ms between two checks if the
 * final position has been reached
 */
KatanaGotoThread::KatanaGotoThread(fawkes::RefPtr<CLMBase> katana,
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
KatanaGotoThread::init() {};

void
KatanaGotoThread::finalize() {};

void
KatanaGotoThread::once()
{
  try {
    // non-blocking call to KNI
    _katana->moveRobotTo(__x, __y, __z, __phi, __theta, __psi);
  } catch (KNI::NoSolutionException &e) {
    _logger->log_warn("KatanaGotoThread", "Initiating goto failed (no solution, ignoring): %s", e.what());
    _finished = true;
    _error_code = fawkes::KatanaInterface::ERROR_NO_SOLUTION;
    return;
  } catch (/*KNI*/::Exception &e) {
    _logger->log_warn("KatanaGotoThread", "Initiating goto failed (ignoring): %s", e.what());
    _finished = true;
    _error_code = fawkes::KatanaInterface::ERROR_CMD_START_FAILED;
    return;
  }

  // Check for finished motion
  bool final = false;
  short num_motors = _katana->getNumberOfMotors();
  CKatBase *base = _katana->GetBase();
  const TKatMOT *mot = base->GetMOT();
  short num_errors  = 0;

  while (! final) {
      usleep(__poll_interval_usec);
      final = true;
      try {
	base->GetSCT()->arr[0].recvDAT(); // update sensor values
	base->recvMPS(); // get position for all motors
	base->recvGMS(); // get status flags for all motors
      } catch (/*KNI*/::Exception &e) {
	if (++num_errors <= 10) {
	  _logger->log_warn("KatanaGripperThread", "Receiving MPS/GMS failed, retrying");
	  continue;
	} else {
	  _logger->log_warn("KatanaGripperThread", "Receiving MPS/GMS failed too often, aborting");
	  _error_code = fawkes::KatanaInterface::ERROR_COMMUNICATION;
	  break;
	}
      }

      for (int i=0; i < num_motors; ++i) {
	if (mot->arr[i].GetPVP()->msf == MSF_MOTCRASHED) {
	  _error_code = fawkes::KatanaInterface::ERROR_MOTOR_CRASHED;
	  break;
	}

	final &= std::abs(mot->arr[i].GetTPS()->tarpos - mot->arr[i].GetPVP()->pos) < 100;
      }
    }

    _logger->log_debug(name(), "Position (%f,%f,%f, %f,%f,%f) reached",
		       __x, __y, __z, __phi, __theta, __psi);

  _finished = true;
}
