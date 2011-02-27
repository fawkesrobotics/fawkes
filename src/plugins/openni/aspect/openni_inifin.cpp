
/***************************************************************************
 *  openni_inifin.cpp - Fawkes OpenNiAspect initializer/finalizer
 *
 *  Created: Sat Feb 26 15:42:56 2011
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

#include <plugins/openni/aspect/openni_inifin.h>
#include <core/threading/thread_finalizer.h>
#include <XnCppWrapper.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class OpenNiAspectIniFin <plugins/openni/aspect/openni_inifin.h>
 * OpenNiAspect initializer/finalizer.
 * This initializer/finalizer will provide the OpenNI context to threads
 * with the OpenNiAspect.
 * @author Tim Niemueller
 */

/** Constructor. */
OpenNiAspectIniFin::OpenNiAspectIniFin()
  : AspectIniFin("OpenNiAspect")
{
}

void
OpenNiAspectIniFin::init(Thread *thread)
{
  OpenNiAspect *openni_thread;
  openni_thread = dynamic_cast<OpenNiAspect *>(thread);
  if (openni_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "OpenNiAspect, but RTTI says it "
					  "has not. ", thread->name());
  }
  if (! __openni_context) {
    throw CannotInitializeThreadException("OpenNI context has not been set.");
  }

  openni_thread->init_OpenNiAspect(__openni_context);
}

void
OpenNiAspectIniFin::finalize(Thread *thread)
{
}


/** Set the OpenNI context to use for aspect initialization.
 * @param openni_context OpenNI context to pass to threads with OpenNiAspect.
 */
void
OpenNiAspectIniFin::set_openni_context(LockPtr<xn::Context> openni_context)
{
  __openni_context = openni_context;
}

} // end namespace fawkes
