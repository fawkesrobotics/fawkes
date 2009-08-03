
/***************************************************************************
 *  employer.cpp - Fawkes main loop employer
 *
 *  Created: Sat Aug  2 00:08:53 2008
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <aspect/mainloop/employer.h>

namespace fawkes {

/** @class MainLoopEmployer <aspect/mainloop/employer.h>
 * Main loop employer
 * The MainLoopEmployer calls the main loop for execution. It is usually
 * implemented in the main program.
 * @author Tim Niemueller
 *
 * @fn void MainLoopEmployer::set_mainloop_thread(fawkes::Thread *mainloop_thread)
 * Set a new main loop.
 * Set the main loop. An exception should be thrown if anything prevents this
 * from happening successful.
 * @param mainloop_thread new main loop thread. The caller of this method must
 * ensure that the thread operates in wait-for-wakeup mode and executes the main
 * loop as its loop() method (i.e. the thread must have a properly initialized
 * MainLoopAspect).
 */

/** Virtual empty destructor. */
MainLoopEmployer::~MainLoopEmployer()
{
}

} // end of namespace fawkes
