
/***************************************************************************
 *  blackboard.cpp - Fawkes BlackBoard Aspect initializer/finalizer
 *
 *  Created: Tue Nov 23 23:06:13 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#include <aspect/inifins/blackboard.h>
#include <aspect/blackboard.h>
#include <blackboard/blackboard.h>
#include <blackboard/ownership.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class BlackBoardAspectIniFin <aspect/inifins/blackboard.h>
 * Initializer/finalizer for the BlackBoardAspect.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param blackboard blackboard instance to pass to threads
 */
BlackBoardAspectIniFin::BlackBoardAspectIniFin(BlackBoard *blackboard)
  : AspectIniFin("BlackBoardAspect")
{
  __blackboard = blackboard;
}

void
BlackBoardAspectIniFin::init(Thread *thread)
{
  BlackBoardAspect *blackboard_thread;
  blackboard_thread = dynamic_cast<BlackBoardAspect *>(thread);
  if (blackboard_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "BlackBoardAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  BlackBoard *bb;
  if (blackboard_thread->blackboard_owner_name_) {
    bb = new BlackBoardWithOwnership(__blackboard,
				     blackboard_thread->blackboard_owner_name_);
  } else {
    bb = new BlackBoardWithOwnership(__blackboard, thread->name());
  }

  blackboard_thread->init_BlackBoardAspect(bb);
}

void
BlackBoardAspectIniFin::finalize(Thread *thread)
{
  BlackBoardAspect *blackboard_thread;
  blackboard_thread = dynamic_cast<BlackBoardAspect *>(thread);
  if (blackboard_thread == NULL) {
    throw CannotFinalizeThreadException("Thread '%s' claims to have the "
					"BlackBoardAspect, but RTTI says it "
					"has not. ", thread->name());
  }

  delete blackboard_thread->blackboard;
  blackboard_thread->blackboard = NULL;
}


} // end namespace fawkes
