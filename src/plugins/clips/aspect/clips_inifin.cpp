
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
#include <plugins/clips/aspect/clips_env_manager.h>
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

/** Destructor. */
CLIPSAspectIniFin::~CLIPSAspectIniFin()
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
  
  LockPtr<CLIPS::Environment> clips =
    clips_env_mgr_->create_env(clips_thread->clips_env_name,
			       clips_thread->CLIPSAspect_log_component_name_);

  clips_thread->clips = clips;
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

  clips_env_mgr_->destroy_env(clips_thread->clips_env_name);
  clips_thread->finalize_CLIPSAspect();
}



/** Set CLIPS environment manger.
 * @param clips_env_mgr CLIPS environment manager
 */
void
CLIPSAspectIniFin::set_manager(LockPtr<CLIPSEnvManager> &clips_env_mgr)
{
  clips_env_mgr_ = clips_env_mgr;
}

} // end namespace fawkes
