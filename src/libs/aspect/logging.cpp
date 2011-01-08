
/***************************************************************************
 *  logging.cpp - Logging aspect for Fawkes
 *
 *  Created: Wed Jan 17 14:30:20 2007
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

#include <aspect/logging.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class LoggingAspect <aspect/logging.h>
 * Thread aspect to log output.
 * Give this aspect to your thread to gain access to the central log.
 * When using this thread all output should be done with the logger set
 * for this aspect. Use it to log debug, informational, warning and error
 * messages. The logger allows for some control over what is being displayed
 * and where. It may be simple console logout or a network logger.
 *
 * It is guaranteed that if used properly from within plugins that
 * initLoggingAspect() is called before the thread is started and that
 * you can access the logger via the logger member.
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */


/** @var Logger LoggingAspect::logger
 * This is the Logger member used to access the logger.
 * The logger will remain valid for the whole lifetime of the
 * thread.
 */

/** Constructor. */
LoggingAspect::LoggingAspect()
{
  add_aspect("LoggingAspect");
}


/** Virtual empty Destructor. */
LoggingAspect::~LoggingAspect()
{
}


/** Set the logger.
 * It is guaranteed that this is called for a logging thread before
 * Thread::start() is called (when running regularly inside Fawkes).
 * @param logger Logger instance to use.
 */
void
LoggingAspect::init_LoggingAspect(Logger *logger)
{
  this->logger = logger;
}

} // end namespace fawkes
