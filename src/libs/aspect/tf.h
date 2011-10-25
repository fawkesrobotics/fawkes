
/***************************************************************************
 *  tf.h - Transform aspect for Fawkes
 *
 *  Created: Tue Oct 25 21:33:21 2011
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

#ifndef __ASPECT_TF_H_
#define __ASPECT_TF_H_

#include <aspect/aspect.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

namespace fawkes {

  class BlackBoard;

#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class TransformAspect : public virtual Aspect
{
 public:
  TransformAspect();
  TransformAspect(const char *tf_bb_iface_id);
  virtual ~TransformAspect();

  void init_TransformAspect(BlackBoard *blackboard);
  void finalize_TransformAspect();

 protected:
  tf::TransformListener    * tf_listener;
  tf::TransformBroadcaster * tf_publisher;

 private:
  char *__tf_aspect_bb_iface_id;
};

} // end namespace fawkes

#endif
