
/***************************************************************************
 *  skel_if_observer.cpp - Skeleton interface observer
 *
 *  Created: Sat Apr 02 18:20:29 2011 (RoboCup German Open 2011, Magdeburg)
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
#include <interfaces/HumanSkeletonInterface.h>
#include <interfaces/HumanSkeletonProjectionInterface.h>
#include <plugins/openni/utils/skel_if_observer.h>

#include <cstdio>

namespace fawkes {
namespace openni {

/** @class SkelIfObserver <plugins/openni/utils/skel_if_observer.h>
 * Skeleton interface observer.
 * This class opens all OpenNI skeleton interfaces and registers as an
 * observer to open any newly opened interface.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param bb blackboard to interact with
 * @param users user map for exchange with others
 */
SkelIfObserver::SkelIfObserver(BlackBoard *bb, UserMap &users) : users_(users)
{
	queue_lock_ = new Mutex();
	bb_         = bb;

	std::list<HumanSkeletonInterface *> skels =
	  bb_->open_multiple_for_reading<HumanSkeletonInterface>("OpenNI Human *");

	std::list<HumanSkeletonProjectionInterface *> projs;

	std::list<HumanSkeletonInterface *>::iterator i;
	for (i = skels.begin(); i != skels.end(); ++i) {
		printf("Opened %s\n", (*i)->uid());

		UserInfo user;
		user.skel_if = *i;
		user.proj_if = bb_->open_for_reading<HumanSkeletonProjectionInterface>(user.skel_if->id());

		users_[user.skel_if->id()] = user;
	}

	bbio_add_observed_create("HumanSkeletonInterface", "OpenNI Human *");
	bb_->register_observer(this);
}

/** Destructor. */
SkelIfObserver::~SkelIfObserver()
{
	bb_->unregister_observer(this);
	delete queue_lock_;
}

void
SkelIfObserver::bb_interface_created(const char *type, const char *id) throw()
{
	if (users_.find(id) == users_.end()) {
		queue_lock_->lock();
		queues_[active_queue_].push(id);
		queue_lock_->unlock();
	}
}

/** Process internal queue.
 * This should be called regularly to process incoming events.
 */
void
SkelIfObserver::process_queue()
{
	queue_lock_->lock();
	unsigned int proc_queue = active_queue_;
	active_queue_           = 1 - active_queue_;
	queue_lock_->unlock();
	while (!queues_[proc_queue].empty()) {
		std::string id = queues_[proc_queue].front();

		try {
			UserInfo user;
			printf("Opening %s\n", id.c_str());
			user.skel_if = bb_->open_for_reading<HumanSkeletonInterface>(id.c_str());
			try {
				user.proj_if = bb_->open_for_reading<HumanSkeletonProjectionInterface>(id.c_str());
			} catch (Exception &e) {
				bb_->close(user.skel_if);
				throw;
			}

			users_[id] = user;
		} catch (Exception &e) {
			e.print_trace();
			continue;
		}

		queues_[proc_queue].pop();
	}
}

} // namespace openni
} // end namespace fawkes
