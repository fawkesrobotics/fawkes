
/***************************************************************************
 *  network.cpp - Fawkes NetworkAspect initializer/finalizer
 *
 *  Created: Tue Nov 23 23:47:59 2010
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

#include <aspect/inifins/network.h>
#include <aspect/network.h>
#include <netcomm/service_discovery/service_publisher.h>
#include <netcomm/service_discovery/service_browser.h>
#include <netcomm/utils/resolver.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NetworkAspectIniFin <aspect/inifins/network.h>
 * Initializer/finalizer for the NetworkAspect.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param nnresolver network name resolver to pass to threads
 * @param service_browser service browser to pass to threads
 * @param service_publisher service publisher resolver to pass to threads
 */
NetworkAspectIniFin::NetworkAspectIniFin(NetworkNameResolver *nnresolver,
					 ServicePublisher *service_publisher,
					 ServiceBrowser *service_browser)
  : AspectIniFin("NetworkAspect")
{
  __nnresolver        = nnresolver;
  __service_publisher = service_publisher;
  __service_browser   = service_browser;
}


void
NetworkAspectIniFin::init(Thread *thread)
{
  NetworkAspect *network_thread;
  network_thread = dynamic_cast<NetworkAspect *>(thread);
  if (network_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "NetworkAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  network_thread->init_NetworkAspect(__nnresolver,
				     __service_publisher, __service_browser);
}


void
NetworkAspectIniFin::finalize(Thread *thread)
{
}


} // end namespace fawkes
