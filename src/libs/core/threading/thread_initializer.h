
/***************************************************************************
 *  thread_initializer.h - Thread initializer interface
 *
 *  Created: Mon Nov 20 00:52:18 2006
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory. 
 */

#ifndef __CORE_THREADING_THREAD_INITIALIZER_H_
#define __CORE_THREADING_THREAD_INITIALIZER_H_

#include <core/exception.h>

class Thread;

class CannotInitializeThreadException : public Exception
{
 public:
  CannotInitializeThreadException(const char *format, ...);
};

class ThreadInitializer
{
 public:
  virtual ~ThreadInitializer();

  virtual void init(Thread *thread) = 0;
};

#endif
