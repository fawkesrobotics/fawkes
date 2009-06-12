
/***************************************************************************
 *  gripper_thread.cpp - Katana gripper one-time thread
 *
 *  Created: Thu Jun 11 11:59:38 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include "gripper_thread.h"

#include <kniBase.h>

/** @class KatanaGripperThread "gripper_thread.h"
 * Katana gripper thread.
 * This thread opens or closes the gripper when started.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param katana katana linear motion base class
 * @param logger logger
 * @param poll_interval_ms interval in ms between two checks if the
 * final position has been reached
 */
KatanaGripperThread::KatanaGripperThread(fawkes::RefPtr<CLMBase> katana,
					 fawkes::Logger *logger,
					 unsigned int poll_interval_ms)
  : KatanaMotionThread("KatanaGripperThread", katana, logger)
{
  __mode               = OPEN_GRIPPER;
  __poll_interval_usec = poll_interval_ms * 1000;
}


/** Set mode.
 * @param mode open, either open or close
 */
void
KatanaGripperThread::set_mode(gripper_mode_t mode)
{
  __mode = mode;
}


void
KatanaGripperThread::once()
{
  try {
    // non-blocking call to KNI
    if (__mode == CLOSE_GRIPPER) {
      _katana->closeGripper(/* wait */ false);
    } else {
      _katana->openGripper(/* wait */ false);
    }

  } catch (/*KNI*/::Exception &e) {
    _logger->log_warn("KatanaGripperThread", "Starting gripper motion failed (ignoring): %s", e.what());
    _finished = true;
    _error_code = fawkes::KatanaInterface::ERROR_CMD_START_FAILED;
    return;
  }

  CKatBase *base = _katana->GetBase();
  const TKatMOT *motors = base->GetMOT();
  CMotBase *gripmot = &motors->arr[motors->cnt - 1];
  const TMotPVP *grippvp = gripmot->GetPVP();
  bool final = false;
  short last_pos    = 0;
  short num_samepos = 0;
  short num_errors  = 0;
  while (! final) {
    try {
      base->GetSCT()->arr[0].recvDAT(); // update sensor values
      gripmot->recvPVP();
    } catch (/*KNI*/::Exception &e) {
      if (++num_errors <= 10) {
	_logger->log_warn("KatanaGripperThread", "Receiving PVP failed, retrying");
	continue;
      } else {
	_logger->log_warn("KatanaGripperThread", "Receiving PVP failed too often, aborting");
	_error_code = fawkes::KatanaInterface::ERROR_COMMUNICATION;
	break;
      }
    }
    if (grippvp->pos == last_pos) {
      if (++num_samepos >= 3) {
	_logger->log_debug(name(), "Gripper did not move for 3 cycles, considering as final");
	final = true;
      }
    } else {
      //_logger->log_debug(name(), "Gripper still moving %i != %i", last_pos, grippvp->pos);
      last_pos = grippvp->pos;
      num_samepos = 0;
      usleep(__poll_interval_usec);
    }
  }

  _logger->log_debug("KatanaGripperThread", "Gripper motion finished");
  _finished = true;
}
