
/***************************************************************************
 *  openrave_inifin.cpp - Fawkes OpenRaveAspect initializer/finalizer
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

#include <plugins/openrave/aspect/openrave_inifin.h>
#include <plugins/openrave/aspect/openrave.h>
#include <plugins/openrave/aspect/openrave_connector.h>

#include <core/threading/thread_finalizer.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class OpenRaveAspectIniFin <plugins/openrave/aspect/openrave_inifin.h>
 * OpenRaveAspect initializer/finalizer.
 * This initializer/finalizer will provide the OpenRaveConnector to threads with
 * the OpenRaveAspect.
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param openrave OpenRaveConnector to pass on to threads
 */
OpenRaveAspectIniFin::OpenRaveAspectIniFin(OpenRaveConnector *openrave)
  : AspectIniFin("OpenRaveAspect")
{
  __openrave = openrave;
}

void
OpenRaveAspectIniFin::init(Thread *thread)
{
  OpenRaveAspect *or_thread;

  or_thread = dynamic_cast<OpenRaveAspect *>(thread);
  if (or_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "OpenRaveAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  or_thread->init_OpenRaveAspect(__openrave);
}

void
OpenRaveAspectIniFin::finalize(Thread *thread)
{
}



} // end namespace fawkes
