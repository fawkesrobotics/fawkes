
/***************************************************************************
 *  logger.cpp - Fawkes LoggerAspect initializer/finalizer
 *
 *  Created: Wed Nov 24 01:17:09 2010
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

#include <aspect/inifins/logger.h>
#include <aspect/logger.h>
#include <logging/logger_employer.h>
#include <core/threading/thread_initializer.h>
#include <core/threading/thread_finalizer.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class LoggerAspectIniFin <aspect/inifins/logger.h>
 * Initializer/finalizer for the LoggerAspect.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param employer logger employer to register loggers to
 */
LoggerAspectIniFin::LoggerAspectIniFin(LoggerEmployer *employer)
  : AspectIniFin("LoggerAspect")
{
  __employer = employer;
}


void
LoggerAspectIniFin::init(Thread *thread)
{
  LoggerAspect *logger_thread;
  logger_thread = dynamic_cast<LoggerAspect *>(thread);
  if (logger_thread == 0) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "LoggerAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  try {
    __employer->add_logger(logger_thread->get_logger());
  } catch (Exception &e) {
    CannotInitializeThreadException ce("Thread has LoggerAspect but Logger "
				       "could not be added.");
    ce.append(e);
    throw ce;
  } catch (...) {
    throw CannotInitializeThreadException("Thread has LoggerAspect but Logger "
					  "could not be added.");
  }
}


void
LoggerAspectIniFin::finalize(Thread *thread)
{
  LoggerAspect *logger_thread;
  logger_thread = dynamic_cast<LoggerAspect *>(thread);
  if (logger_thread == 0) {
    throw CannotFinalizeThreadException("Thread '%s' claims to have the "
					"LoggerAspect, but RTTI says it "
					"has not. ", thread->name());
  }

  try {
    __employer->remove_logger(logger_thread->get_logger());
  } catch (Exception &e) {
    CannotFinalizeThreadException ce("Failed to remove logger");
    ce.append(e);
    throw;
  }
}

} // end namespace fawkes
