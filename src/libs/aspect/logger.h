
/***************************************************************************
 *  logger.h - Logger aspect for Fawkes
 *
 *  Created: Wed Feb 11 22:21:24 2009
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

#ifndef __ASPECT_LOGGER_H_
#define __ASPECT_LOGGER_H_

#include <aspect/aspect.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Logger;

class LoggerAspect : public virtual Aspect
{
 public:
  LoggerAspect(Logger *logger) __attribute__((nonnull));
  virtual ~LoggerAspect();

  Logger *  get_logger() const;

 private:
  Logger *__logger;
};

} // end namespace fawkes

#endif
