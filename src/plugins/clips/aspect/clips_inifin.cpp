
/***************************************************************************
 *  clips_inifin.cpp - Fawkes CLIPSAspect initializer/finalizer
 *
 *  Created: Sat Jun 16 14:34:27 2012
 *  Copyright  2006-2012  Tim Niemueller [www.niemueller.de]
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

#include <plugins/clips/aspect/clips_inifin.h>
#include <core/threading/thread_finalizer.h>
#include <clipsmm.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CLIPSAspectIniFin <plugins/clips/aspect/clips_inifin.h>
 * CLIPSAspect initializer/finalizer.
 * This initializer/finalizer will provide the CLIPS node handle to
 * threads with the CLIPSAspect.
 * @author Tim Niemueller
 */

/** Constructor. */
CLIPSAspectIniFin::CLIPSAspectIniFin()
  : AspectIniFin("CLIPSAspect")
{
}

void
CLIPSAspectIniFin::init(Thread *thread)
{
  CLIPSAspect *clips_thread;
  clips_thread = dynamic_cast<CLIPSAspect *>(thread);
  if (clips_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "CLIPSAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  // CLIPS overwrites the SIGINT handler, restore it after
  // initializing the environment
  struct sigaction oldact;
  if (sigaction(SIGINT, NULL, &oldact) == 0) {
    LockPtr<CLIPS::Environment> clips(new CLIPS::Environment());
    clips_thread->init_CLIPSAspect(clips);
    // restore old action
    sigaction(SIGINT, &oldact, NULL);
  } else {
    throw CannotInitializeThreadException("CLIPS for %s: Unable to backup "
                                          "SIGINT sigaction for restoration.",
                                          thread->name());
  }
}

void
CLIPSAspectIniFin::finalize(Thread *thread)
{
  CLIPSAspect *clips_thread;
  clips_thread = dynamic_cast<CLIPSAspect *>(thread);
  if (clips_thread == NULL) {
    throw CannotFinalizeThreadException("Thread '%s' claims to have the "
					"CLIPSAspect, but RTTI says it "
					"has not. ", thread->name());
  }
  clips_thread->finalize_CLIPSAspect();
}

} // end namespace fawkes
