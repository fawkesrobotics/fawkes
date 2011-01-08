
/***************************************************************************
 *  fawkes_network.cpp - Fawkes network aspect for Fawkes
 *
 *  Created: Mon May 07 19:45:32 2007
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

#include <aspect/fawkes_network.h>

#include <netcomm/fawkes/hub.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FawkesNetworkAspect <aspect/fawkes_network.h>
 * Thread aspect to participate in the Fawkes Network protocol.
 * Give this aspect to your thread to make use of the Fawkes network
 * protocol. The protocol is designed to transport short messages over
 * the network (information and control data).
 * With this aspect you can easily inject messages into the stream and
 * and receive messages from connected clients. The component ID has to
 * be unique. To ensure this have a look at netcomm/fawkes/component_ids.h.
 *
 * It is guaranteed that if used properly from within plugins that
 * initFawkesNetworkAspect() is called before the thread is started and that
 * you can access the Fawkes network hub via the fnethub member.
 *
 * In most cases you should implement Thread::init() to register
 * your FawkesNetworkHandler.
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */


/** @var FawkesNetworkHub FawkesNetworkAspect::fnethub
 * This is the Fawkes network hub member used to access the Fawkes
 * network protocol.
 * The logger will remain valid for the whole lifetime of the
 * thread.
 */

/** Constructor. */
FawkesNetworkAspect::FawkesNetworkAspect()
{
  add_aspect("FawkesNetworkAspect");
}

/** Virtual empty Destructor. */
FawkesNetworkAspect::~FawkesNetworkAspect()
{
}


/** Set the logger.
 * It is guaranteed that this is called for a thread having the
 * Fawkes netwok aspect before Thread::start() is called (when
 * running regularly inside Fawkes).
 * @param fnethub Fawkes network hub instance to use for network
 * communication.
 */
void
FawkesNetworkAspect::init_FawkesNetworkAspect(FawkesNetworkHub *fnethub)
{
  this->fnethub = fnethub;
}

} // end namespace fawkes
