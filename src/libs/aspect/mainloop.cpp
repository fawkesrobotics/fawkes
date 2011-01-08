
/***************************************************************************
 *  mainloop.cpp - Main loop aspect for Fawkes
 *
 *  Created: Sat Aug 02 00:16:30 2008
 *  Copyright  2008-2010  Tim Niemueller [www.niemueller.de]
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

#include <aspect/mainloop.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class MainLoopAspect <aspect/mainloop.h>
 * Thread aspect that allows to replace the main loop of the main application
 * of Fawkes.
 * Warning, replacing the main loop may severely interfere with the
 * functionality of Fawkes. Make sure that you know what the main loop
 * needs, what it has to do and what it should not do.
 *
 * At any given time there can only be one thread active with this aspect.
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */

/** @var BlockedTimingExecutor *  MainLoopAspect::blocked_timing_executor
 * This is a blocked timing executor instance which can be used to run threads
 * with the BlockedTimingAspect.
 */

/** Constructor. */
MainLoopAspect::MainLoopAspect()
{
  add_aspect("MainLoopAspect");
}

/** Virtual empty destructor. */
MainLoopAspect::~MainLoopAspect()
{
}


/** Initialize main loop aspect.
 * Called from the Aspect initializer.
 * @param btexec blocked timing executor instance that can be used to run
 * threads that have the blocked timing aspect. It's accessible as
 * blocked_timing_aspect.
 */
void
MainLoopAspect::init_MainLoopAspect(BlockedTimingExecutor *btexec)
{
  blocked_timing_executor = btexec;
}

} // end namespace fawkes
