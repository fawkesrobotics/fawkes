
/***************************************************************************
 *  network.cpp - network aspect for Fawkes
 *
 *  Created: Fri Jun 29 15:17:08 2007 (on flight to RoboCup 2007, Atlanta)
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

#include <aspect/network.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NetworkAspect <aspect/network.h>
 * Thread aspect for network communication.
 * Give this aspect to your thread if you want to implement custom network
 * communication. With this aspect you get access to the central network name
 * resolver and you may publish service on the network and browse for existing
 * services (for example using mDNS-SD via Avahi).
 *
 * It is guaranteed that if used properly from within plugins that
 * init_NetworkAspect() is called before the thread is started.
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */


/** @var NetworkNameResolver NetworkAspect::nnresolver
 * Network name resolver to lookup IP addresses of hostnames and vice versa.
 * The nnresolver will remain valid for the whole lifetime of the
 * thread.
 */

/** @var NetworkNameResolver NetworkAspect::service_publisher
 * Service publisher to publish services on the network.
 * The service_publisher will remain valid for the whole lifetime of the
 * thread.
 */

/** @var NetworkNameResolver NetworkAspect::service_browser
 * Service browser to browse services on the network.
 * The service_browser will remain valid for the whole lifetime of the
 * thread.
 */

/** Constructor. */
NetworkAspect::NetworkAspect()
{
  add_aspect("NetworkAspect");
}

/** Virtual empty Destructor. */
NetworkAspect::~NetworkAspect()
{
}


/** Init network aspect.
 * It is guaranteed that this is called for a thread having the
 * netwok aspect before Thread::start() is called (when
 * running regularly inside Fawkes).
 * @param resolver network name resolver
 * @param service_publisher service publisher
 * @param service_browser service browser
 */
void
NetworkAspect::init_NetworkAspect(NetworkNameResolver *resolver,
				  ServicePublisher *service_publisher,
				  ServiceBrowser *service_browser)
{
  this->nnresolver = resolver;
  this->service_publisher = service_publisher;
  this->service_browser = service_browser;
}

} // end namespace fawkes
