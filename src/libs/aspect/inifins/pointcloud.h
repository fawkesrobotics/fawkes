
/***************************************************************************
 *  pcl.h - Fawkes PointCloudAspect initializer/finalizer
 *
 *  Created: Mon Nov 07 13:36:46 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __ASPECT_INIFINS_PCL_H_
#define __ASPECT_INIFINS_PCL_H_

#include <aspect/inifins/inifin.h>
#include <pcl_utils/pointcloud_manager.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class PointCloudAspectIniFin : public AspectIniFin
{
 public:
  PointCloudAspectIniFin(Configuration *config);
  virtual ~PointCloudAspectIniFin();

  virtual void init(Thread *thread);
  virtual void finalize(Thread *thread);

 private:
  PointCloudManager *__pcl_manager;
};

} // end namespace fawkes

#endif
