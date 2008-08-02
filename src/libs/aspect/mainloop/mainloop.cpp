
/***************************************************************************
 *  mainloop.cpp - Fawkes main loop interface
 *
 *  Created: Sat Aug  2 00:03:17 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <aspect/mainloop/mainloop.h>

namespace fawkes {

/** @class MainLoop <aspect/mainloop/mainloop.h>
 * Main loop class.
 * This class defines a main loop suitable for being used with the MainLoop
 * aspect. If a MainLoopEmployer has been set in the AspectIniFin instance then
 * the main loop can be replaced.
 * @author Tim Niemueller
 *
 * @fn void MainLoop::mloop()
 * Main loop execution.
 * This method is executed for each loop. It has to implement the main loop
 * logic.
 */

/** @var BlockedTimingExecutor * MainLoop::_btexec
 * Blocked timing instance of this main loop. Initialized by the
 * MainLoopAspect during aspect initialization.
 */

/** Virtual empty destructor. */
MainLoop::~MainLoop()
{
}

/** Set blocked timing executor.
 * @param btexec blocked timing executor
 */
void
MainLoop::set_blocked_timing_executor(BlockedTimingExecutor *btexec)
{
  _btexec = btexec;
}

} // end of namespace fawkes
