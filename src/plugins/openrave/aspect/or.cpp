
/***************************************************************************
 *  or.cpp - OpenRAVE aspect for Fawkes
 *
 *  Created: Fri Feb 25 15:08:00 2011
 *  Copyright  2011  Bahram Maleki-Fard
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

#include <plugins/openrave/aspect/or.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class OpenRAVEAspect <plugins/openrave/aspect/or.h>
 * Thread aspect create, update, and graph round-robin databases (RRD).
 * Give this aspect to your thread to access the OpenRAVE environment,
 * add robots or objects, path plans for manipulator movement, etc.
 *
 * @ingroup Aspects
 * @author Bahram Maleki-Fard
 */

/** @var fawkes::OpenRAVEManager *  OpenRAVEAspect::or_manager
 * Manager class to access OpenRAVE features. It will take care of properly
 * distributing the work.
 */

/** Constructor. */
OpenRAVEAspect::OpenRAVEAspect()
{
  add_aspect("OpenRAVEAspect");
}


/** Virtual empty destructor. */
OpenRAVEAspect::~OpenRAVEAspect()
{
}


/** Init OpenRAVE aspect.
 * This sets the OpenRAVE manager to access OpenRAVE.
 * It is guaranteed that this is called for an OpenRAVE Thread before start
 * is called (when running regularly inside Fawkes).
 * @param or_manager OpenRAVEManager to use
 */
void
OpenRAVEAspect::init_OpenRAVEAspect(OpenRAVEConnector *openrave)
{
  this->openrave = openrave;
}

} // end namespace fawkes
