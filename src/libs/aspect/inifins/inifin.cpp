
/***************************************************************************
 *  inifin.cpp - Fawkes Aspect initializer/finalizer base class
 *
 *  Created: Tue Nov 23 23:01:23 2010
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

#include <aspect/inifins/inifin.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class AspectIniFin <aspect/inifins/inifin.h>
 * Aspect initializer/finalizer base class.
 * This class must be derived for each aspect that is added to the system,
 * either standard or custom aspects.
 * @author Tim Niemueller
 *
 * @fn void AspectIniFin::init(Thread *thread)
 * Initialize thread.
 * The aspect for the given thread must be initialized. Use dynamic_cast
 * to cast the thread into the expected aspect class. An exception must
 * be thrown if this fails. If anything fails during initialization of
 * the aspect an Exception must be thrown.
 * @param thread thread to initialize
 *
 * @fn void AspectIniFin::finalize(Thread *thread)
 * Finalize thread.
 * The aspect for the given thread must be initialized. Use dynamic_cast
 * to cast the thread into the expected aspect class. An exception must
 * be thrown if this fails. If anything fails during initialization of
 * the aspect an Exception must be thrown. This will not prevent the
 * thread from being removed. Use prepare_finalize() to report problems
 * that should prevent the thread from being unloaded.
 * @param thread thread to finalize
 */

/** Constructor.
 * @param aspect_name name of the aspect the aspect initializer/finalizer
 * subclass is used for. It must exist for the whole lifetime of the
 * initializer/finalizer.
 */
AspectIniFin::AspectIniFin(const char *aspect_name)
{
  __aspect_name = aspect_name;
}

/** Virtual empty destructor. */
AspectIniFin::~AspectIniFin()
{
}

/** Default finalize preparation.
 * This is a default implementation that assumes that finalization is
 * always safe. Override it if you need to make more fine-grained
 * decisions.
 * @param thread thread to prepare for finalization
 * @return always true
 */
bool
AspectIniFin::prepare_finalize(Thread *thread)
{
  return true;
}

/** Get aspect name.
 * @return name of the aspect this initializer/finalizer is used for
 */
const char *
AspectIniFin::get_aspect_name() const
{
  return __aspect_name;
}

} // end namespace fawkes
