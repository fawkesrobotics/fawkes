
/***************************************************************************
 *  recursive_mutex.h - Recursive Mutex
 *
 *  Generated: Mon Oct 24 18:33:41 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __CORE_THREADING_RECURSIVE_MUTEX_H_
#define __CORE_THREADING_RECURSIVE_MUTEX_H_

#include <core/threading/mutex.h>

namespace fawkes {

class RecursiveMutex : public Mutex
{
  friend class WaitCondition;

 public:
  RecursiveMutex();
};

} // end namespace fawkes

#endif
