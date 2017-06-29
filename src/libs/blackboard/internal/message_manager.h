
/***************************************************************************
 *  message_manager.h - BlackBoard message manager
 *
 *  Created: Fri Oct 06 11:29:51 2006
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __BLACKBOARD_MESSAGE_MANAGER_H_
#define __BLACKBOARD_MESSAGE_MANAGER_H_

#include <interface/mediators/message_mediator.h>

namespace fawkes {

class BlackBoardInterfaceManager;
class BlackBoardNotifier;
class LocalBlackBoard;
class Message;

class BlackBoardMessageManager : public MessageMediator
{
 friend LocalBlackBoard;
 public:
  BlackBoardMessageManager(BlackBoardNotifier *notifier);
  ~BlackBoardMessageManager();

  virtual void transmit(Message *message);

 private:
  BlackBoardInterfaceManager *__im;
  BlackBoardNotifier         *__notifier;

  void set_interface_manager(BlackBoardInterfaceManager *im);
};

} // end namespace fawkes

#endif
