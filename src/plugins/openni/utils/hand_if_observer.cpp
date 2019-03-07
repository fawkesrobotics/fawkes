
/***************************************************************************
 *  hand_if_observer.cpp - Skeleton hand interface observer
 *
 *  Created: Sat Apr 02 19:39:31 2011 (RoboCup German Open 2011, Magdeburg)
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

#include <blackboard/blackboard.h>
#include <interfaces/ObjectPositionInterface.h>
#include <plugins/openni/utils/hand_if_observer.h>

namespace fawkes {
namespace openni {

/** @class HandIfObserver <plugins/openni/utils/hand_if_observer.h>
 * Hand interface observer.
 * This class opens all OpenNI hand interfaces and registers as an
 * observer to open any newly opened interface.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param bb blackboard to interact with
 * @param hands hand map for exchange with others
 */
HandIfObserver::HandIfObserver(BlackBoard *bb, HandMap &hands) : hands_(hands)
{
	queue_lock_ = new Mutex();
	bb_         = bb;

	std::list<ObjectPositionInterface *> hand_ifs =
	  bb_->open_multiple_for_reading<ObjectPositionInterface>("OpenNI Hand *");

	std::list<ObjectPositionInterface *>::iterator i;
	for (i = hand_ifs.begin(); i != hand_ifs.end(); ++i) {
		HandInfo hand;
		hand.hand_if               = *i;
		hands_[hand.hand_if->id()] = hand;
	}

	bbio_add_observed_create("ObjectPositionInterface", "OpenNI Hand *");
	bb_->register_observer(this);
}

/** Destructor. */
HandIfObserver::~HandIfObserver()
{
	bb_->unregister_observer(this);
	delete queue_lock_;
}

void
HandIfObserver::bb_interface_created(const char *type, const char *id) throw()
{
	if (hands_.find(id) == hands_.end()) {
		queue_lock_->lock();
		queues_[active_queue_].push(id);
		queue_lock_->unlock();
	}
}

/** Process internal queue.
 * This should be called regularly to process incoming events.
 */
void
HandIfObserver::process_queue()
{
	queue_lock_->lock();
	unsigned int proc_queue = active_queue_;
	active_queue_           = 1 - active_queue_;
	queue_lock_->unlock();
	while (!queues_[proc_queue].empty()) {
		std::string id = queues_[proc_queue].front();

		try {
			HandInfo hand;
			hand.hand_if = bb_->open_for_reading<ObjectPositionInterface>(id.c_str());

			hands_[id] = hand;
		} catch (Exception &e) {
			e.print_trace();
			continue;
		}

		queues_[proc_queue].pop();
	}
}

} // namespace openni
} // end namespace fawkes
