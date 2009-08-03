
/***************************************************************************
 *  stream.h - Fawkes stream socket (TCP)
 *
 *  Created: Fri Nov 10 10:01:22 2006 (on train to Google, Hamburg)
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __NETCOMM_SOCKET_STREAM_H_
#define __NETCOMM_SOCKET_STREAM_H_

#include <netcomm/socket/socket.h>

namespace fawkes {

class StreamSocket : public Socket
{
 public:
  StreamSocket(float timeout = 0.f);
  StreamSocket(StreamSocket &s);

  virtual Socket *  clone();

  void set_nodelay(bool no_delay);
  bool nodelay();

 protected:
};

} // end namespace fawkes

#endif
