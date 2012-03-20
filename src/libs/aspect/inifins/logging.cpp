
/***************************************************************************
 *  logging.cpp - Fawkes Logging Aspect initializer/finalizer
 *
 *  Created: Tue Nov 23 23:06:13 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#include <aspect/inifins/logging.h>
#include <aspect/logging.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class LoggingAspectIniFin <aspect/inifins/logging.h>
 * Initializer/finalizer for the LoggingAspect.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param logger logger instance to pass to threads
 */
LoggingAspectIniFin::LoggingAspectIniFin(Logger *logger)
  : AspectIniFin("LoggingAspect")
{
  __logger = logger;
}


void
LoggingAspectIniFin::init(Thread *thread)
{
  LoggingAspect *logging_thread;
  logging_thread = dynamic_cast<LoggingAspect *>(thread);
  if (logging_thread == 0) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "LoggingAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  logging_thread->init_LoggingAspect(__logger);
}


void
LoggingAspectIniFin::finalize(Thread *thread)
{
}


} // end namespace fawkes
