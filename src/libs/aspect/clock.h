
/***************************************************************************
 *  clock.h - Clock aspect for Fawkes
 *
 *  Created: Tue Jun 12 18:36:09 2007
 *  Copyright  2007       Daniel Beck
 *             2007-2010  Tim Niemueller [www.niemueller.de]
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

#ifndef __ASPECT_CLOCK_H_
#define __ASPECT_CLOCK_H_

#include <aspect/aspect.h>
#include <utils/time/clock.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ClockAspect : public virtual Aspect
{
 public:
  ClockAspect();
  virtual ~ClockAspect();

  void init_ClockAspect(Clock *clock);

 protected:
  Clock *clock;
};

} // end namespace fawkes

#endif /*__ASPECT_CLOCK_H_ */
