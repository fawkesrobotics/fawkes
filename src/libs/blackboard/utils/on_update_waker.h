
/***************************************************************************
 *  on_update_waker.h - wake a thread whenever am interface is updated
 *
 *  Created: Thu Dec 06 14:26:01 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#ifndef _BLACKBOARD_UTILS_ON_UPDATE_WAKER_H_
#define _BLACKBOARD_UTILS_ON_UPDATE_WAKER_H_

#include <blackboard/interface_listener.h>

namespace fawkes {

class Interface;
class Thread;

class BlackBoardOnUpdateWaker : public BlackBoardInterfaceListener
{
public:
	BlackBoardOnUpdateWaker(BlackBoard *bb, Interface *interface, Thread *thread);
	virtual ~BlackBoardOnUpdateWaker();

	virtual void bb_interface_data_refreshed(Interface *interface) throw();

private:
	BlackBoard *bb_;
	Thread *    thread_;
};

} // end namespace fawkes

#endif
