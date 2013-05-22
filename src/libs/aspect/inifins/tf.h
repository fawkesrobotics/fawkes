
/***************************************************************************
 *  tf.h - Fawkes TransformAspect initializer/finalizer
 *
 *  Created: Tue Oct 25 22:31:12 2011
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

#ifndef __ASPECT_INIFINS_TF_H_
#define __ASPECT_INIFINS_TF_H_

#include <aspect/inifins/inifin.h>

namespace fawkes {
  namespace tf {
    class Transformer;
  }

#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class BlackBoard;

class TransformAspectIniFin : public AspectIniFin
{
 public:
  TransformAspectIniFin(BlackBoard *blackboard, tf::Transformer *transformer);

  virtual void init(Thread *thread);
  virtual void finalize(Thread *thread);

 private:
  BlackBoard      *__blackboard;
  tf::Transformer *__transformer;
};

} // end namespace fawkes

#endif
