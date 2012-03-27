
/***************************************************************************
 *  pointcloud.cpp - PointCloud aspect for Fawkes
 *
 *  Created: Sun Nov 06 22:51:58 2011
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

#include <aspect/pointcloud.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class PointCloudAspect <aspect/pointcloud.h>
 * Thread aspect to provide and access point clouds.
 *
 * TODO
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */


/** Constructor. */
PointCloudAspect::PointCloudAspect()
{
  add_aspect("PointCloudAspect");
}


/** Virtual empty Destructor. */
PointCloudAspect::~PointCloudAspect()
{
}


/** Set URL manager.
 * It is guaranteed that this is called for a thread before
 * Thread::start() is called (when running regularly inside Fawkes).
 * @param cloud_manager the cloud manager to distribute clouds
 */
void
PointCloudAspect::init_PointCloudAspect(PointCloudManager *cloud_manager)
{
  pcl_manager = cloud_manager;
}

} // end namespace fawkes
