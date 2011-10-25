/***************************************************************************
 *  transform_broadcaster_protector.cpp - Transform broadcaster protector
 *
 *  Created: Tue Oct 25 22:17:48 2011
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

#include <tf/transform_broadcaster_protector.h>

namespace fawkes {
  namespace tf {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

/** @class TransformBroadcasterProtector <tf/transform_broadcaster_protector.h>
 * Utility class to avoid null pointer errors in TransformAspect.
 * This class will throw an exception if you try to send a transform.
 * It is used in the TransformAspect to avoid segfaults due to an
 * uninitialized broadcaster if the wrong constructor has been used.
 * @author Tim Niemueller
 */

/** Destructor. */
TransformBroadcasterProtector::~TransformBroadcasterProtector()
{
}


/** Publish transform.
 * @param transform transform to publish
 */
void
TransformBroadcasterProtector::send_transform(const StampedTransform &transform)
{
  throw Exception("Transform broadcaster has not been requested. "
                  "Used wrong TransformAspect constructor?");
}

} // end namespace tf
} // end namespace fawkes
