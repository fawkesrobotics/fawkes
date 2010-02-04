
/***************************************************************************
 *  fuse_client_handler.h - FUSE network client handler
 *
 *  Created: Wed Nov 14 16:47:56 2007
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

#ifndef __FIREVISION_FVUTILS_NET_FUSE_CLIENT_HANDLER_H_
#define __FIREVISION_FVUTILS_NET_FUSE_CLIENT_HANDLER_H_

#include <fvutils/net/fuse.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class FuseNetworkMessage;

class FuseClientHandler {
 public:
  virtual ~FuseClientHandler();

  virtual void fuse_invalid_server_version(uint32_t local_version,
					   uint32_t remote_version) throw() = 0;
  virtual void fuse_connection_established() throw() = 0;
  virtual void fuse_connection_died() throw() = 0;
  virtual void fuse_inbound_received(FuseNetworkMessage *m) throw() = 0;
};


} // end namespace firevision

#endif
