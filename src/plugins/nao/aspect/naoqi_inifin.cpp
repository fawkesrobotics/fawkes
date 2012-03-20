
/***************************************************************************
 *  naoqi_inifin.cpp - Fawkes NaoQiAspect initializer/finalizer
 *
 *  Created: Thu May 12 15:54:02 2011
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

#include <plugins/nao/aspect/naoqi_inifin.h>
#include <core/threading/thread_finalizer.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NaoQiAspectIniFin <plugins/nao/aspect/naoqi_inifin.h>
 * NaoQiAspect initializer/finalizer.
 * This initializer/finalizer will provide the NaoQi broker to threads
 * with the NaoQiAspect.
 * @author Tim Niemueller
 */

/** Constructor. */
NaoQiAspectIniFin::NaoQiAspectIniFin()
  : AspectIniFin("NaoQiAspect")
{
}

void
NaoQiAspectIniFin::init(Thread *thread)
{
  NaoQiAspect *naoqi_thread;
  naoqi_thread = dynamic_cast<NaoQiAspect *>(thread);
  if (naoqi_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "NaoQiAspect, but RTTI says it "
					  "has not. ", thread->name());
  }
  if (! __naoqi_broker) {
    throw CannotInitializeThreadException("NaoQi broker has not been set.");
  }

  naoqi_thread->init_NaoQiAspect(__naoqi_broker);
}

void
NaoQiAspectIniFin::finalize(Thread *thread)
{
}


/** Set the NaoQi broker to use for aspect initialization.
 * @param naoqi_broker NaoQi broker to pass to threads with NaoQiAspect.
 */
void
NaoQiAspectIniFin::set_naoqi_broker(AL::ALPtr<AL::ALBroker> naoqi_broker)
{
  __naoqi_broker = naoqi_broker;
}

} // end namespace fawkes
