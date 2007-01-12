
/***************************************************************************
 *  message_mediator.h - Fawkes BlackBoard Message Mediator
 *
 *  Generated: Sun Oct 29 17:58:19 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __MESSAGE_MEDIATOR_H_
#define __MESSAGE_MEDIATOR_H_

class Interface;
class Message;

class MessageMediator
{
 public:
  /** Virtual destructor */
  virtual ~MessageMediator() {}

  /** Transmit message.
   * @param message message to transmit.
   */
  virtual void transmit(Message *message)                              = 0;

};

#endif
