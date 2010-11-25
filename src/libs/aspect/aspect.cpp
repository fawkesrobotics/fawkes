
/***************************************************************************
 *  aspect.cpp - Aspect base class
 *
 *  Created: Tue Nov 23 22:27:43 2010
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

#include <aspect/aspect.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class Aspect <aspect/aspect.h>
 * Fawkes aspect base class.
 * This base class is the core for providing an extensible aspects system.
 * Aspects inherit from this base class via virtual inheritance. That means
 * that the constructor is only called once, and hence we can keep a list of
 * names of the aspects attached to a thread. This way we can easily
 * recognize all aspects of a thread, even though the aspect might currently
 * be unknown to the system, because it has not been registered.
 *
 * Do not use this class directly for anything other than creating a new
 * aspect.
 *
 * @author Tim Niemueller
 */

/** Add an aspect to a thread.
 * This records the name of the threads added to a thread. This method may
 * must be used exactly once in constructors of aspects, and only there.
 * @param name aspect name that is added to the thread
 */
void
Aspect::add_aspect(const char *name)
{
  __aspects.push_back(name);
}


/** Get list of aspect names attached to a aspected thread.
 * @return list of aspect names attached to an aspected thread
 */
const std::list<const char *> &
Aspect::get_aspects() const
{
  return __aspects;
}


} // end namespace fawkes
