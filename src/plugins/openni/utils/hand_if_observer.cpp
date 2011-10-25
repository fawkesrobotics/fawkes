
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

#include <plugins/openni/utils/hand_if_observer.h>

#include <blackboard/blackboard.h>
#include <interfaces/ObjectPositionInterface.h>

namespace fawkes {
  namespace openni {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

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
HandIfObserver::HandIfObserver(BlackBoard *bb, HandMap &hands)
  : __hands(hands)
{
  __queue_lock = new Mutex();
  __bb = bb;

  std::list<ObjectPositionInterface *> hand_ifs =
    __bb->open_multiple_for_reading<ObjectPositionInterface>("OpenNI Hand *");

  std::list<ObjectPositionInterface *>::iterator i;
  for (i = hand_ifs.begin(); i != hand_ifs.end(); ++i) {
    HandInfo hand;
    hand.hand_if = *i;
    __hands[hand.hand_if->id()] = hand;
  }

  bbio_add_observed_create("ObjectPositionInterface", "OpenNI Hand *");
  __bb->register_observer(this);
}


/** Destructor. */
HandIfObserver::~HandIfObserver()
{
  __bb->unregister_observer(this);
  delete __queue_lock;
}

void
HandIfObserver::bb_interface_created(const char *type, const char *id) throw()
{
  if (__hands.find(id) == __hands.end()) {
    __queue_lock->lock();
    __queues[__active_queue].push(id);
    __queue_lock->unlock();
  }
}

/** Process internal queue.
 * This should be called regularly to process incoming events.
 */
void
HandIfObserver::process_queue()
{
  __queue_lock->lock();
  unsigned int proc_queue = __active_queue;
  __active_queue = 1 - __active_queue;
  __queue_lock->unlock();
  while (! __queues[proc_queue].empty()) {
    std::string id = __queues[proc_queue].front();

    try {
      HandInfo hand;
      hand.hand_if = __bb->open_for_reading<ObjectPositionInterface>(id.c_str());

      __hands[id] = hand;
    } catch (Exception &e) {
      e.print_trace();
      continue;
    }

    __queues[proc_queue].pop();
  }
}

} // end namespace fawkes::openni
} // end namespace fawkes
