
/***************************************************************************
 *  openprs_manager_inifin.cpp - OpenPRSManagerAspect initializer/finalizer
 *
 *  Created: Mon Aug 18 15:21:24 2014
 *  Copyright  2006-2013  Tim Niemueller [www.niemueller.de]
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

#include <plugins/openprs/aspect/openprs_manager_inifin.h>
#include <plugins/openprs/aspect/openprs_kernel_manager.h>
#include <core/threading/thread_finalizer.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class OpenPRSManagerAspectIniFin <plugins/clips/aspect/clips_feature_inifin.h>
 * OpenPRSManagerAspect initializer/finalizer.
 * @author Tim Niemueller
 */

/** Constructor. */
OpenPRSManagerAspectIniFin::OpenPRSManagerAspectIniFin()
  : AspectIniFin("OpenPRSManagerAspect")
{
}

/** Destructor. */
OpenPRSManagerAspectIniFin::~OpenPRSManagerAspectIniFin()
{
}



void
OpenPRSManagerAspectIniFin::init(Thread *thread)
{
  OpenPRSManagerAspect *openprs_thread;
  openprs_thread = dynamic_cast<OpenPRSManagerAspect *>(thread);
  if (openprs_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "OpenPRSManagerAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  openprs_thread->openprs_kernel_mgr = openprs_kernel_mgr_;
}


void
OpenPRSManagerAspectIniFin::finalize(Thread *thread)
{
  OpenPRSManagerAspect *openprs_thread;
  openprs_thread = dynamic_cast<OpenPRSManagerAspect *>(thread);
  if (openprs_thread == NULL) {
    throw CannotFinalizeThreadException("Thread '%s' claims to have the "
					"OpenPRSManagerAspect, but RTTI says it "
					"has not. ", thread->name());
  }

  openprs_thread->openprs_kernel_mgr.clear();
}



/** Set OpenPRS environment manger.
 * @param openprs_kernel_mgr OpenPRS kernel manager
 */
void
OpenPRSManagerAspectIniFin::set_manager(LockPtr<OpenPRSKernelManager> &openprs_kernel_mgr)
{
  openprs_kernel_mgr_ = openprs_kernel_mgr;
}

} // end namespace fawkes
