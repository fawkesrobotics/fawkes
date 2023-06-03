
/***************************************************************************
 *  hand_if_observer.h - Skeleton hand interface observer
 *
 *  Created: Sat Apr 02 19:26:57 2011 (RoboCup German Open 2011, Magdeburg)
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef _PLUGINS_OPENNI_UTILS_HAND_IF_OBSERVER_H_
#define _PLUGINS_OPENNI_UTILS_HAND_IF_OBSERVER_H_

#include <blackboard/interface_observer.h>
#include <plugins/openni/utils/types.h>

#include <queue>
#include <string>

namespace fawkes {
class BlackBoard;
class Mutex;

namespace openni {

class HandIfObserver : public BlackBoardInterfaceObserver
{
public:
	HandIfObserver(BlackBoard *bb, HandMap &hands);
	~HandIfObserver();

	virtual void bb_interface_created(const char *type, const char *id) noexcept;

	void process_queue();

private:
	HandMap                &hands_;
	BlackBoard             *bb_;
	Mutex                  *queue_lock_;
	unsigned int            active_queue_;
	std::queue<std::string> queues_[2];
};

} // namespace openni
} // end namespace fawkes

#endif
