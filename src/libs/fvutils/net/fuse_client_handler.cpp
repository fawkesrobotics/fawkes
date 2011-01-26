
/***************************************************************************
 *  fuse_client_handler.cpp - FUSE network client handler
 *
 *  Created: Wed Nov 14 16:48:42 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/net/fuse_client_handler.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FuseClientHandler <fvutils/net/fuse_client_handler.h>
 * FUSE client handler.
 * @ingroup FUSE
 * @ingroup FireVision
 * @author Tim Niemueller
 *
 * @fn void FuseClientHandler::fuse_invalid_server_version(uint32_t local_version, uint32_t remote_version) throw() = 0
 * Invalid version string received.
 * The remote end has a different incompatible FUSE version.
 * @param local_version version that the FuseClient speaks
 * @param remote_version version that the remote FUSE server speaks.
 *
 * @fn void FuseClientHandler::fuse_connection_established() throw() = 0
 * Connection has been established.
 *
 * @fn void FuseClientHandler::fuse_connection_died()  throw() = 0
 * Connection died.
 *
 *
 * @fn void FuseClientHandler::fuse_inbound_received(FuseNetworkMessage *m) throw() = 0
 * Message received.
 * An incoming message has been received and can now be processed. Note that if you want
 * to work on the message after this method has finished you have to reference the message
 * by calling FuseMessage::ref()!
 * @param m message to handle
 */


/** Destructor. */
FuseClientHandler::~FuseClientHandler()
{
}

} // end namespace firevision
