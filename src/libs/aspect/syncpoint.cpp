/***************************************************************************
 *  syncpoint.cpp - SyncPoint Aspect
 *
 *  Created: Thu Feb 19 14:31:42 2015
 *  Copyright  2015  Till Hofmann
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

#include <aspect/syncpoint.h>

#include <core/threading/thread.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class SyncPointAspect <aspect/syncpoint_manager.h>
 * Thread aspect to acces to SyncPoints
 * Give this aspect to your thread to manage SyncPoints,
 * i.e. wait for SyncPoints and emit SyncPoints
 * @ingroup Aspects
 * @author Till Hofmann
 */

/** Constructor.
 * Use this constructor if there should be an input syncpoint. The input syncpoint
 * will be waited for before every loop.
 * @param type_in type of the input syncpoint
 * @param identifier_in identifier of the input syncpoint
 * @param identifier_out identifier of the output syncpoint.
 *          If this identifier is empty, no output syncpoint will be used.
 */
SyncPointAspect::SyncPointAspect(SyncPoint::WakeupType type_in, std::string identifier_in,
    std::string identifier_out /* = "" */)
  : type_in_(type_in), identifier_in_(identifier_in),
    identifier_out_(identifier_out), sp_in_(NULL), sp_out_(NULL)
{
  add_aspect("SyncPointAspect");
  has_input_syncpoint_ = (identifier_in != "");
  has_output_syncpoint_ = (identifier_out != "");
}

/** Constructor.
 * Use this constructor if there should be no input syncpoint, but only an output
 * syncpoint.
 * @param identifier_out identifier of the output syncpoint
 */
SyncPointAspect::SyncPointAspect(std::string identifier_out)
  : type_in_(SyncPoint::NONE), identifier_in_(""),
    identifier_out_(identifier_out), sp_in_(NULL), sp_out_(NULL)
{
  add_aspect("SyncPointAspect");
  has_input_syncpoint_ = false;
  has_output_syncpoint_ = true;
}

/** Destructor */
SyncPointAspect::~SyncPointAspect()
{
}

/** Init SyncPoint aspect.
 * This initializes the syncpoints and registers the thread as loop listener.
 * Additionally, the thread is registered as emitter for the output syncpoint
 * if an output syncpoint is created.
 * @param thread thread which uses this aspect
 * @param manager SyncPointManager to use
 */
void
SyncPointAspect::init_SyncPointAspect(Thread *thread, SyncPointManager *manager)
{
  if (has_input_syncpoint_) {
    sp_in_ = manager->get_syncpoint(thread->name(), identifier_in_);
  }

  if (has_output_syncpoint_) {
    sp_out_ = manager->get_syncpoint(thread->name(), identifier_out_);
    sp_out_->register_emitter(thread->name());
  }

  if (has_input_syncpoint_ || has_output_syncpoint_) {
    thread->add_loop_listener(this);
  }
}

/** Finalize SyncPoint aspect.
 * This releases all syncpoints and unregisters the thread as loop listener.
 * @param thread thread which uses this aspect
 * @param manager SyncPointManager to use
 */
void
SyncPointAspect::finalize_SyncPointAspect(Thread *thread, SyncPointManager *manager)
{
  if (has_input_syncpoint_) {
    manager->release_syncpoint(thread->name(), sp_in_);
  }

  if (has_output_syncpoint_) {
    sp_out_->unregister_emitter(thread->name());
    manager->release_syncpoint(thread->name(), sp_out_);
  }

  if (has_input_syncpoint_ || has_output_syncpoint_) {
    thread->remove_loop_listener(this);
  }
}

/** Wait for the input syncpoint before loop()
 * @param thread thread which uses this aspect and whose loop() will be called
 */
void
SyncPointAspect::pre_loop(Thread *thread)
{
  if (has_input_syncpoint_) {
    sp_in_->wait(thread->name(), type_in_);
  }
}

/** Emit the output syncpoint after loop()
 * @param thread thread which uses this aspect and whose loop() just returned
 */
void
SyncPointAspect::post_loop(Thread *thread)
{
  if (has_output_syncpoint_) {
    sp_out_->emit(thread->name());
  }
}

} // end namespace fawkes
