
/***************************************************************************
 *  skel_if_observer.h - Skeleton interface observer
 *
 *  Created: Sat Apr 02 18:14:37 2011 (RoboCup German Open 2011, Magdeburg)
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

#ifndef __PLUGINS_OPENNI_UTILS_SKEL_IF_OBSERVER_H_
#define __PLUGINS_OPENNI_UTILS_SKEL_IF_OBSERVER_H_

#include <plugins/openni/utils/types.h>
#include <blackboard/interface_observer.h>

#include <queue>
#include <string>

namespace fawkes {
  class BlackBoard;
  class Mutex;

  namespace openni {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

class SkelIfObserver : public BlackBoardInterfaceObserver
{
 public:
  SkelIfObserver(BlackBoard *bb, UserMap &users);
  ~SkelIfObserver();

  virtual void bb_interface_created(const char *type, const char *id) throw();

  void process_queue();

 private:
  UserMap &__users;
  BlackBoard *__bb;
  Mutex *__queue_lock;
  unsigned int __active_queue;
  std::queue<std::string> __queues[2];
};

} // end namespace fawkes::openni
} // end namespace fawkes

#endif
