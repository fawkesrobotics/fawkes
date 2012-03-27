
/***************************************************************************
 *  pcl.h - Convert PCL buffer to PointCloud structure
 *
 *  Created: Wed Nov 02 19:11:47 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_UTILS_ADAPTERS_PCL_H
#define __FIREVISION_UTILS_ADAPTERS_PCL_H

#ifdef HAVE_PCL
#  include <pcl/point_cloud.h>
#  include <pcl/point_types.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class SharedMemoryImageBuffer;

void convert_buffer_to_pcl(const SharedMemoryImageBuffer *buffer,
                           pcl::PointCloud<pcl::PointXYZ> &pcl);

void convert_buffer_to_pcl(const SharedMemoryImageBuffer *buffer,
                           pcl::PointCloud<pcl::PointXYZRGB> &pcl);

} // end namespace firevision
#else // not HAVE_PCL
#  error PCL not available, guard your include with HAVE_PCL
#endif
#endif
