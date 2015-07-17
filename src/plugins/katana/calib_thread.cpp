
/***************************************************************************
 *  calib_thread.cpp - Katana calibration one-time thread
 *
 *  Created: Tue Jun 09 18:37:02 2009
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

#include "calib_thread.h"
#include "controller.h"

#include <interfaces/KatanaInterface.h>

/** @class KatanaCalibrationThread "calib_thread.h"
 * Katana calibration thread.
 * This thread calibrates the arm when started and is finished when the calibration
 * is done.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param katana katana controller base class
 * @param logger logger
 */
KatanaCalibrationThread::KatanaCalibrationThread(fawkes::RefPtr<fawkes::KatanaController> katana,
						 fawkes::Logger *logger)
  : KatanaMotionThread("KatanaCalibrationThread", katana, logger)
{
}

void
KatanaCalibrationThread::once()
{
  try {
    _katana->calibrate();
    _logger->log_debug(name(), "Calibration successful");
  } catch (fawkes::Exception &e) {
    _logger->log_warn(name(), "Calibration failed (ignoring error): %s", e.what());
    _error_code = fawkes::KatanaInterface::ERROR_CMD_START_FAILED;
  }
  _finished = true;
}
