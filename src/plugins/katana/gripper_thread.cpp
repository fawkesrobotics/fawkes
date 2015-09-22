
/***************************************************************************
 *  gripper_thread.cpp - Katana gripper one-time thread
 *
 *  Created: Thu Jun 11 11:59:38 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *             2012-2014  Bahram Maleki-Fard
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
#include "controller.h"
#include "exception.h"

#include <interfaces/KatanaInterface.h>

#include <unistd.h>

/** @class KatanaGripperThread "gripper_thread.h"
 * Katana gripper thread.
 * This thread opens or closes the gripper when started.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param katana katana controller base class
 * @param logger logger
 * @param poll_interval_ms interval in ms between two checks if the
 * final position has been reached
 */
KatanaGripperThread::KatanaGripperThread(fawkes::RefPtr<fawkes::KatanaController> katana,
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
    // non-blocking call
    if (__mode == CLOSE_GRIPPER) {
      _katana->gripper_close(/* wait */ false);
    } else {
      _katana->gripper_open(/* wait */ false);
    }

  } catch (fawkes::Exception &e) {
    _logger->log_warn("KatanaGripperThread", "Starting gripper motion failed (ignoring): %s", e.what());
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

  _logger->log_debug("KatanaGripperThread", "Gripper motion finished");

  _finished = true;
}
