
/***************************************************************************
 *  employer.h - Fawkes logger employer
 *
 *  Created: Wed Feb 11 22:26:27 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __ASPECT_LOGGER_EMPLOYER_H_
#define __ASPECT_LOGGER_EMPLOYER_H_

namespace fawkes {

class Logger;

class LoggerEmployer
{
 public:
  virtual ~LoggerEmployer();

  virtual void add_logger(Logger *logger)    = 0;
  virtual void remove_logger(Logger *logger) = 0;
};

} // end of namespace fawkes

#endif
