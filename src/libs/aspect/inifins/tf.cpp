
/***************************************************************************
 *  tf.cpp - Fawkes TransformAspect initializer/finalizer
 *
 *  Created: Tue Oct 25 22:32:59 2011
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

#include <aspect/inifins/tf.h>
#include <aspect/tf.h>

namespace fawkes {

  namespace tf {
    class Transformer;
  }

#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class TransformAspectIniFin <aspect/inifins/tf.h>
 * Initializer/finalizer for the TransformAspect.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param blackboard blackboard instance to pass to threads
 * @param transformer system-wide shared transformer to pass to threads
 */
TransformAspectIniFin::TransformAspectIniFin(BlackBoard *blackboard,
                                             tf::Transformer *transformer)
  : AspectIniFin("TransformAspect")
{
  __blackboard  = blackboard;
  __transformer = transformer;
}

void
TransformAspectIniFin::init(Thread *thread)
{
  TransformAspect *transform_thread;
  transform_thread = dynamic_cast<TransformAspect *>(thread);
  if (transform_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "TransformAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  transform_thread->init_TransformAspect(__blackboard, __transformer, thread->name());
}

void
TransformAspectIniFin::finalize(Thread *thread)
{
  TransformAspect *transform_thread;
  transform_thread = dynamic_cast<TransformAspect *>(thread);
  if (transform_thread == NULL) {
    throw CannotFinalizeThreadException("Thread '%s' claims to have the "
                                        "TransformAspect, but RTTI says it "
                                        "has not. ", thread->name());
  }

  transform_thread->finalize_TransformAspect();
}


} // end namespace fawkes
