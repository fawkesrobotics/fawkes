/***************************************************************************
 *  transform_broadcaster.h - Fawkes transform broadcaster (based on ROS tf)
 *
 *  Created: Mon Oct 24 17:10:30 2011
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

#ifndef __LIBS_TF_TRANSFORM_BROADCASTER_PROTECTOR_H_
#define __LIBS_TF_TRANSFORM_BROADCASTER_PROTECTOR_H_

#include <tf/transform_broadcaster.h>

namespace fawkes {
  namespace tf {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

class TransformBroadcasterProtector : public TransformBroadcaster
{
 public:
  TransformBroadcasterProtector() {};
  virtual ~TransformBroadcasterProtector();
  virtual void send_transform(const StampedTransform &transform);
};


} // end namespace tf
} // end namespace fawkes

#endif
