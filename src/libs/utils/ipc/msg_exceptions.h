
/***************************************************************************
 *  msg_exceptions.h - exceptions thrown in msg utils, do NOT put your own
 *                     application specific exceptions here!
 *
 *  Generated: Mon Sep 18 23:11:29 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_IPC_MSG_EXCEPTIONS_H_
#define __UTILS_IPC_MSG_EXCEPTIONS_H_

#include <core/exception.h>

namespace fawkes {


/** Message did not fit into buffer. */
class MessageTooBigException : public Exception {
 public:
  /** Constructor */
  MessageTooBigException() : Exception("Message too big for buffer")  {}
};


} // end namespace fawkes

#endif
