
/***************************************************************************
 *  fawkes_network.cpp - Fawkes FawkesNetworkAspect initializer/finalizer
 *
 *  Created: Tue Nov 23 23:44:10 2010
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

#include <aspect/inifins/fawkes_network.h>
#include <aspect/fawkes_network.h>
#include <netcomm/fawkes/hub.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FawkesNetworkAspectIniFin <aspect/inifins/fawkes_network.h>
 * Initializer/finalizer for the FawkesNetworkAspect.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param hub Fawkes network hub instance to pass to threads
 */
FawkesNetworkAspectIniFin::FawkesNetworkAspectIniFin(FawkesNetworkHub *hub)
  : AspectIniFin("FawkesNetworkAspect")
{
  __hub = hub;
}


void
FawkesNetworkAspectIniFin::init(Thread *thread)
{
  FawkesNetworkAspect *fnet_thread;
  fnet_thread = dynamic_cast<FawkesNetworkAspect *>(thread);
  if (fnet_thread == 0) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "FawkesNetworkAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  fnet_thread->init_FawkesNetworkAspect(__hub);
}


void
FawkesNetworkAspectIniFin::finalize(Thread *thread)
{
}


} // end namespace fawkes
