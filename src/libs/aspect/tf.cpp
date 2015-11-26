
/***************************************************************************
 *  tf.cpp - Transform aspect for Fawkes
 *
 *  Created: Tue Oct 25 21:35:14 2011
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

#include <aspect/tf.h>
#include <tf/transform_listener.h>

#include <cstring>
#include <cstdlib>
#include <cstdarg>
#include <core/threading/thread_initializer.h>
#include <core/exceptions/system.h>
#include <blackboard/ownership.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class TransformAspect <aspect/tf.h>
 * Thread aspect to access the transform system.

 * Give this aspect to your thread to gain access to the transform
 * library.  Depending on the parameters to the ctor only the listener
 * or additionaly the publisher is created.
 * It is guaranteed that if used properly from within plugins that the
 * blackboard member has been initialized properly.
 * @ingroup Aspects
 * @author Tim Niemueller
 */


/** @var tf::TransformListener *  TransformAspect::tf_listener
 * This is the transform listener which saves transforms published by
 * other threads in the system.
 */

/** @var tf::TransformPublisher *  TransformAspect::tf_publisher
 * This is the transform publisher which can be used to publish
 * transforms via the blackboard. It is only created if the constructor
 * taking the blackboard interface ID parameter is used!
 */

/** @var std::map<std::string, tf::TransformPublisher *>  TransformAspect::tf_publishers
 * Map of transform publishers created through the aspect.

 * The maps key is the blackboard interface ID passed to either the
 * constructor or tf_add_publisher(). The ID is used as passed, i.e.,
 * not with the /tf/ prefix which might be added by the
 * TransformPublisher. The singular tf_publisher is also added to the
 * map.
 */

/** Constructor.
 * @param mode mode of operation
 * @param frame_id ID of frame to create publisher for, can be zero if
 * creating of publisher is omitted or deferred.
 */
TransformAspect::TransformAspect(Mode mode, const char *frame_id)
  : __tf_aspect_mode(mode)
{
  add_aspect("TransformAspect");
  if (((mode == ONLY_PUBLISHER) || (mode == BOTH) ||
       (mode == BOTH_DEFER_PUBLISHER) || (mode == DEFER_PUBLISHER))
      && frame_id)
  {
	  __tf_aspect_frame_id = strdup(frame_id);
  } else {
	  __tf_aspect_frame_id = 0;
  }
  __tf_aspect_blackboard = 0;
}


/** Virtual empty destructor. */
TransformAspect::~TransformAspect()
{
  if (__tf_aspect_frame_id)  free(__tf_aspect_frame_id);
}


/** Init transform aspect.
 * This creates the listener and potentially publisher.
 * @param blackboard blackboard used to create listener and/or publisher.
 * @param transformer system-wide shared transformer to pass to threads
 * @param thread_name name of thread opening publishers
 */
void
TransformAspect::init_TransformAspect(BlackBoard *blackboard, tf::Transformer *transformer,
                                      const char *thread_name)
{
  if (((__tf_aspect_mode == ONLY_PUBLISHER) || (__tf_aspect_mode == BOTH)) &&
      (__tf_aspect_frame_id == NULL))
  {
    throw CannotInitializeThreadException("TransformAspect was initialized "
                                          "in mode %s but BB interface ID"
                                          "is not set",
                                          (__tf_aspect_mode == BOTH) ? "BOTH"
                                          : "ONLY_PUBLISHER");
  }

  __tf_aspect_blackboard = new BlackBoardWithOwnership(blackboard, thread_name);

  if ((__tf_aspect_mode == ONLY_LISTENER) || (__tf_aspect_mode == BOTH) ||
      (__tf_aspect_mode == BOTH_DEFER_PUBLISHER))
  {
    tf_listener = transformer;
  } else {
    tf_listener = NULL;
  }

  if ((__tf_aspect_mode == ONLY_PUBLISHER) || (__tf_aspect_mode == BOTH)) {
    tf_publisher =
      new tf::TransformPublisher(__tf_aspect_blackboard, __tf_aspect_frame_id);
    tf_publishers[__tf_aspect_frame_id] = tf_publisher;
  } else {
    tf_publisher = new tf::TransformPublisher(NULL, NULL);
  }
}


/** Late enabling of publisher.
 * If and only if the TransformAspect has been initialized in
 * DEFER_PUBLISHER or BOTH_DEFER_PUBLISHER mode the transform
 * publisher can be enabled using this method. It will create a new
 * transform publisher with the interface ID given as constructor
 * parameter.
 *
 * This method is intended to be used if it is unclear at construction
 * time whether the publisher will be needed or not.
 * @param frame_id Frame ID to use for publisher. This can only be passed if
 * the frame_id passed to the constructor was null.
 * @exception Exception thrown if the TransformAspect is not initialized in
 * DEFER_PUBLISHER or BOTH_DEFER_PUBLISHER mode.
 */
void
TransformAspect::tf_enable_publisher(const char *frame_id)
{
  if ((__tf_aspect_mode != DEFER_PUBLISHER) && (__tf_aspect_mode != BOTH_DEFER_PUBLISHER)) {
    throw Exception("Publisher can only be enabled later in (BOTH_)DEFER_PUBLISHER mode");
  }
  if (frame_id) {
	  if (__tf_aspect_frame_id) {
		  throw Exception("Cannot overwrite frame_id '%s' with '%s' in tf_enable_publisher",
		                  __tf_aspect_frame_id, frame_id);
	  } else {
		  __tf_aspect_frame_id = strdup(frame_id);
	  }
  }
  if (__tf_aspect_frame_id == 0) {
	  throw Exception("TransformAspect in %s mode "
	                  "requires a valid blackboard interface ID to enable the publisher",
	                  __tf_aspect_mode == DEFER_PUBLISHER
	                  ? "DEFER_PUBLISHER" : "BOTH_DEFER_PUBLISHER" );
  }

  delete tf_publisher;
  tf_publisher =
    new tf::TransformPublisher(__tf_aspect_blackboard, __tf_aspect_frame_id);
  tf_publishers[__tf_aspect_frame_id] = tf_publisher;
}


/** Late add of publisher.
 * If and only if the TransformAspect has been initialized in
 * DEFER_PUBLISHER or BOTH_DEFER_PUBLISHER mode additional transform
 * publishers can be added using this method. It will create a new
 * transform publisher with the given interface ID.
 *
 * This method is intended to be used if it is unclear at construction
 * time whether the publisher will be needed or not.
 * @exception Exception thrown if the TransformAspect is not initialized in
 * DEFER_PUBLISHER or BOTH_DEFER_PUBLISHER mode.
 * @param frame_id_format format string of interface ID to create. See man printf
 * for accepted patterns. If string starts with / is taken as is, otherwise "/tf/" is
 * prepended to interface ID implicitly.
 */
void
TransformAspect::tf_add_publisher(const char *frame_id_format, ...)
{
  if ((__tf_aspect_mode != DEFER_PUBLISHER) && (__tf_aspect_mode != BOTH_DEFER_PUBLISHER)) {
    throw Exception("Publisher can only be enabled later in (BOTH_)DEFER_PUBLISHER mode");
  }

  va_list arg;
  va_start(arg, frame_id_format);

  char *msg;
  if (vasprintf(&msg, frame_id_format, arg) == -1) {
    throw OutOfMemoryException("Cannot format transform publisher BB interface ID");
  }
  va_end(arg);
  std::string frame_id = msg;
  free(msg);

  if (tf_publishers.find(frame_id) != tf_publishers.end()) {
	  throw Exception("Publisher for %s has already been added", frame_id.c_str());
  }

  tf_publishers[frame_id] =
	  new tf::TransformPublisher(__tf_aspect_blackboard, frame_id.c_str());
}

/** Finalize transform aspect.
 * This deletes the transform listener and publisher.
 */
void
TransformAspect::finalize_TransformAspect()
{
  if (__tf_aspect_frame_id) {
	  tf_publishers.erase(__tf_aspect_frame_id);
  }
  delete tf_publisher;
  std::map<std::string, tf::TransformPublisher *>::iterator ti;
  for (ti = tf_publishers.begin(); ti != tf_publishers.end(); ++ti) {
	  delete ti->second;
  }
  tf_publishers.clear();
  tf_listener = 0;
  tf_publisher = 0;
  delete __tf_aspect_blackboard;
  __tf_aspect_blackboard = 0;
}

} // end namespace fawkes
