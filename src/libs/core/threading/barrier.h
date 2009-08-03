
/***************************************************************************
 *  barrier.h - Barrier
 *
 *  Created: Thu Sep 15 00:31:50 2006
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __CORE_THREADING_BARRIER_H_
#define __CORE_THREADING_BARRIER_H_

namespace fawkes {


class BarrierData;

class Barrier
{
 public:
  Barrier(unsigned int count);
  virtual ~Barrier();

  virtual void wait();

  unsigned int count();

 private:
  BarrierData *barrier_data;

 protected:
  Barrier();
  /** Number of threads that are expected to wait for the barrier */
  unsigned int _count;
};


} // end namespace fawkes

#endif
