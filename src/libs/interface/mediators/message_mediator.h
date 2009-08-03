
/***************************************************************************
 *  message_mediator.h - Fawkes BlackBoard Message Mediator
 *
 *  Created: Sun Oct 29 17:58:19 2006
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __MESSAGE_MEDIATOR_H_
#define __MESSAGE_MEDIATOR_H_

namespace fawkes {

class Interface;
class Message;

/** Message mediator interface.
 * The message mediator is used by an interface to communicate messages to the
 * BlackBoard for dispatching.
 * @author Tim Niemueller
 */
class MessageMediator
{
 public:
  /** Virtual destructor */
  virtual ~MessageMediator() {}

  /** Transmit message.
   * The mediator may modify the message ID of the message.
   * @param message message to transmit.
   * @exception BlackBoardNoWritingInstanceException thrown if there is no writing
   *instance for the transmitting interface
   */
  virtual void transmit(Message *message)                        = 0;

};

} // end namespace fawkes

#endif
