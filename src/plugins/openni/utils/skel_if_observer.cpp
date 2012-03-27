
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

#include <plugins/openni/utils/skel_if_observer.h>

#include <blackboard/blackboard.h>
#include <interfaces/HumanSkeletonInterface.h>
#include <interfaces/HumanSkeletonProjectionInterface.h>

#include <cstdio>

namespace fawkes {
  namespace openni {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

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
SkelIfObserver::SkelIfObserver(BlackBoard *bb, UserMap &users)
  : __users(users)
{
  __queue_lock = new Mutex();
  __bb = bb;

  std::list<HumanSkeletonInterface *> skels =
    __bb->open_multiple_for_reading<HumanSkeletonInterface>("OpenNI Human *");

  std::list<HumanSkeletonProjectionInterface *> projs;

  std::list<HumanSkeletonInterface *>::iterator i;
  for (i = skels.begin(); i != skels.end(); ++i) {
    printf("Opened %s\n", (*i)->uid());

    UserInfo user;
    user.skel_if = *i;
    user.proj_if =
      __bb->open_for_reading<HumanSkeletonProjectionInterface>(user.skel_if->id());

    __users[user.skel_if->id()] = user;
  }

  bbio_add_observed_create("HumanSkeletonInterface", "OpenNI Human *");
  __bb->register_observer(this);
}


/** Destructor. */
SkelIfObserver::~SkelIfObserver()
{
  __bb->unregister_observer(this);
  delete __queue_lock;
}

void
SkelIfObserver::bb_interface_created(const char *type, const char *id) throw()
{
  if (__users.find(id) == __users.end()) {
    __queue_lock->lock();
    __queues[__active_queue].push(id);
    __queue_lock->unlock();
  }
}

/** Process internal queue.
 * This should be called regularly to process incoming events.
 */
void
SkelIfObserver::process_queue()
{
  __queue_lock->lock();
  unsigned int proc_queue = __active_queue;
  __active_queue = 1 - __active_queue;
  __queue_lock->unlock();
  while (! __queues[proc_queue].empty()) {
    std::string id = __queues[proc_queue].front();

    try {
      UserInfo user;
      printf("Opening %s\n", id.c_str());
      user.skel_if = __bb->open_for_reading<HumanSkeletonInterface>(id.c_str());
      try {
	user.proj_if =
	  __bb->open_for_reading<HumanSkeletonProjectionInterface>(id.c_str());
      } catch (Exception &e) {
	__bb->close(user.skel_if);
	throw;
      }

      __users[id] = user;
    } catch (Exception &e) {
      e.print_trace();
      continue;
    }

    __queues[proc_queue].pop();
  }
}

} // end namespace fawkes::openni
} // end namespace fawkes
