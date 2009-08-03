
/***************************************************************************
 *  refcount.h - reference counting base class
 *
 *  Created: Fri Oct 27 13:43:50 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __CORE_UTILS_REFCOUNT_H_
#define __CORE_UTILS_REFCOUNT_H_

namespace fawkes {


class Mutex;

class RefCount
{
 public:
  RefCount();
  virtual ~RefCount();

  void              ref();
  void              unref();
  unsigned int      refcount();

 private:

  unsigned int  refc;
  Mutex        *ref_mutex;
};


} // end namespace fawkes

#endif
