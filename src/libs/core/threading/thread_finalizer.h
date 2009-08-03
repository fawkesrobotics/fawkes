
/***************************************************************************
 *  thread_finalizer.h - Thread finalizer interface
 *
 *  Created: Wed May 23 14:00:57 2007 (Illuminatus day)
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __CORE_THREADING_THREAD_FINALIZER_H_
#define __CORE_THREADING_THREAD_FINALIZER_H_

#include <core/exception.h>

namespace fawkes {


class Thread;

class CannotFinalizeThreadException : public Exception
{
 public:
  CannotFinalizeThreadException(const char *format, ...);
  CannotFinalizeThreadException(Exception &e);
};

class ThreadFinalizer
{
 public:
  virtual ~ThreadFinalizer();

  virtual bool prepare_finalize(Thread *thread)                      = 0;
  virtual void finalize(Thread *thread)                              = 0;
};


} // end namespace fawkes

#endif
