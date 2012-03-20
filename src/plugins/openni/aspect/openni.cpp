
/***************************************************************************
 *  openni.cpp - OpenNI aspect for Fawkes
 *
 *  Created: Sat Feb 26 15:36:58 2011
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

#include <plugins/openni/aspect/openni.h>
#include <XnCppWrapper.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class OpenNiAspect <plugins/openni/aspect/openni.h>
 * Thread aspect to get access to the OpenNI context.
 * Give this aspect to your thread to interact with the central OpenNI
 * context. Use this as a last resort, first check if the desired information
 * is available via the blackboard (e.g. user skeleton data) or via
 * FireVision (camera images).
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */

/** @var fawkes:LockPtr<xn::Context> OpenNiAspect::openni
 * Central OpenNI context. Make sure you use proper locking in your application
 * when using the class, or chaos and havoc will come upon you.
 */

/** Constructor. */
OpenNiAspect::OpenNiAspect()
{
  add_aspect("OpenNiAspect");
}


/** Virtual empty destructor. */
OpenNiAspect::~OpenNiAspect()
{
}


/** Init OpenNI aspect.
 * This set the OpenNI context.
 * It is guaranteed that this is called for an OpenNI Thread before start
 * is called (when running regularly inside Fawkes).
 * @param openni_context OpenNI context to use
 */
void
OpenNiAspect::init_OpenNiAspect(LockPtr<xn::Context> openni_context)
{
  this->openni = openni_context;
}

} // end namespace fawkes
