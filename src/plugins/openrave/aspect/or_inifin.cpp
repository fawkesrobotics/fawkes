
/***************************************************************************
 *  or_inifin.cpp - Fawkes OpenRAVEAspect initializer/finalizer
 *
 *  Created: Fri Feb 25 15:08:00 2011
 *  Copyright  2011  Bahram Maleki-Fard
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

#include <plugins/openrave/aspect/or_inifin.h>
#include <plugins/openrave/aspect/or.h>
#include <plugins/openrave/aspect/or_connector.h>

#include <core/threading/thread_finalizer.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class OpenRAVEAspectIniFin <plugins/rrd/aspect/rrd_inifin.h>
 * OpenRAVEAspect initializer/finalizer.
 * This initializer/finalizer will provide the OpenRAVEManager to threads with
 * the OpenRAVEAspect.
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param or_manager OpenRAVEManager to pass on to threads
 */
OpenRAVEAspectIniFin::OpenRAVEAspectIniFin(OpenRAVEConnector *openrave)
  : AspectIniFin("OpenRAVEAspect")
{
  __openrave = openrave;
}

void
OpenRAVEAspectIniFin::init(Thread *thread)
{
  OpenRAVEAspect *or_thread;
  or_thread = dynamic_cast<OpenRAVEAspect *>(thread);
  if (or_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "OpenRAVEAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  or_thread->init_OpenRAVEAspect(__openrave);
}

void
OpenRAVEAspectIniFin::finalize(Thread *thread)
{
}



} // end namespace fawkes
