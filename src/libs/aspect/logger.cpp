
/***************************************************************************
 *  logger.cpp - Logger aspect for Fawkes
 *
 *  Created: Wed Feb 11 22:21:56 2009
 *  Copyright  2008-2009  Tim Niemueller [www.niemueller.de]
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

#include <aspect/logger.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class LoggerAspect <aspect/logger.h>
 * Thread aspect that allows to provide a logger to Fawkes.
 * The logger will be added to the list of loggers and will get all the
 * log messages.
 * This aspect can be used for example to attach Fawkes to a custom logging
 * facility.
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */


/** Constructor.
 * @param logger Logger to add to the Fawkes loggers
 */
LoggerAspect::LoggerAspect(Logger *logger)
{
  add_aspect("LoggerAspect");
  __logger = logger;
}

/** Virtual empty destructor. */
LoggerAspect::~LoggerAspect()
{
}


/** Get time source.
 * This method is called by the aspect initializer to get the time source
 * the thread with this aspect provides.
 * @return logger provide by the thread with this aspect
 */
Logger *
LoggerAspect::get_logger() const
{
  return __logger;
}

} // end namespace fawkes
