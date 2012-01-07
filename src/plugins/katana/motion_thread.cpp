
/***************************************************************************
 *  motion_thread.h - Katana one-time thread interface for motions
 *
 *  Created: Wed Jun 10 11:41:36 2009
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

#include "motion_thread.h"
#include "controller.h"

/** @class KatanaMotionThread "motion_thread.h"
 * Katana motion thread base class.
 * Base class for motion threads for the Katana.
 *
 * When implementing a motion thread ensure that you read the sensor data
 * during the communication with the arm. The main (act) thread will not do
 * this as not to interfere with motion thread communication. You can use
 * code like this:
 * @code
 * _katana->GetBase()->GetSCT()->arr[0].recvDAT(); // update sensor values
 * @endcode
 * @author Tim Niemueller.
 */

/** Constructor.
 * @param thread_name name of the thread
 * @param katana katana controller base class
 * @param logger logger
 */
KatanaMotionThread::KatanaMotionThread(const char * thread_name,
				       fawkes::RefPtr<fawkes::KatanaController> katana,
				       fawkes::Logger *logger)
  : Thread(thread_name, Thread::OPMODE_CONTINUOUS)
{
  _katana     = katana;
  _logger     = logger;
  _finished   = false;
  _error_code = 0;
}


/** Did the motion finish already?
 * @return true if the motion was finished, flase otherwise
 */
bool
KatanaMotionThread::finished() const
{
  return _finished;
}


/** Error code.
 * @return error code, one or more of the ERROR_* constants from the
 * KatanaInterface or'ed.
 */
unsigned int
KatanaMotionThread::error_code() const
{
  return _error_code;
}


/** Reset for next execution.
 * Resets _finished and _error_code. If you override this method call the base
 * class method in your method. It should be used to do anything that is required
 * to be able to run the thread again.
 */
void
KatanaMotionThread::reset()
{
  _finished   = false;
  _error_code = 0;
}
