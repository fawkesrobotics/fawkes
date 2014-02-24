
/***************************************************************************
 *  factory.h - Logger factory
 *
 *  Created: Mon Jun 04 10:54:35 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_LOGGING_FACTORY_H_
#define __UTILS_LOGGING_FACTORY_H_

#include <core/exception.h>
#include <core/exceptions/software.h>
#include <logging/logger.h>

#include <cstddef>

namespace fawkes {


class UnknownLoggerTypeException : public Exception
{
 public:
  UnknownLoggerTypeException(const char *msg = NULL);
};

class MultiLogger;

class LoggerFactory
{
 public:
  static Logger * instance(const char *type, const char *as);
  static MultiLogger *multilogger_instance(const char *as,
					   Logger::LogLevel default_ll = Logger::LL_DEBUG);

  /** Get typed instance of logger.
   * Creates a new instance and converts it to the requested type. If the type
   * does not match the requested logger an exception is thrown.
   * @param type logger type
   * @param as logger argument string
   * @return typed logger instance
   * @exception TypeMismatchException thrown, if requested logger does not match
   * requested type.
   */
  template <class L>
    static L* instance(const char *type, const char *as);

 private:
  static Logger::LogLevel string_to_loglevel(const char *log_level);
};


template <class L>
L *
LoggerFactory::instance(const char *type, const char *as)
{
  Logger *l = LoggerFactory::instance(type, as);
  L *tl = dynamic_cast<L *>(l);
  if ( tl == NULL ) {
    throw TypeMismatchException("Named type %s is not template type", type);
  }
  return tl;
}

} // end namespace fawkes

#endif
