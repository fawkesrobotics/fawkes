
/***************************************************************************
 *  pcl.cpp - Fawkes PointCloudAspect initializer/finalizer
 *
 *  Created: Mon Nov 07 13:37:38 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include <aspect/inifins/pointcloud.h>
#include <aspect/pointcloud.h>
#include <pcl_utils/utils.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class PointCloudAspectIniFin <aspect/inifins/pointcloud.h>
 * Initializer/finalizer for the PointCloudAspect.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config configuration used to conditionally make PCL quiet.
 */
PointCloudAspectIniFin::PointCloudAspectIniFin(Configuration *config)
  : AspectIniFin("PointCloudAspect")
{
  __pcl_manager = new PointCloudManager();
  pcl_utils::shutup_conditional(config);
}


/** Destructor. */
PointCloudAspectIniFin::~PointCloudAspectIniFin()
{
  delete __pcl_manager;
}

void
PointCloudAspectIniFin::init(Thread *thread)
{
  PointCloudAspect *pcl_thread;
  pcl_thread = dynamic_cast<PointCloudAspect *>(thread);
  if (pcl_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "PointCloudAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  pcl_thread->init_PointCloudAspect(__pcl_manager);
}

void
PointCloudAspectIniFin::finalize(Thread *thread)
{
}


} // end namespace fawkes
