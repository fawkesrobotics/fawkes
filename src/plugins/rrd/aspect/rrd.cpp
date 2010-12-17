
/***************************************************************************
 *  rrd.cpp - RRD aspect for Fawkes
 *
 *  Created: Fri Dec 17 00:21:28 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#include <plugins/rrd/aspect/rrd.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class RRDAspect <plugins/rrd/aspect/rrd.h>
 * Thread aspect create, update, and graph round-robin databases (RRD).
 * Give this aspect to your thread to create, update, and graph round-robin
 * databases produced by rrdtool.
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */

/** @var fawkes::RRDManager *  RRDAspect::rrd_manager
 * Manager class to access RRD features. It will take care of properly
 * distributing the work.
 */

/** Constructor. */
RRDAspect::RRDAspect()
{
  add_aspect("RRDAspect");
}


/** Virtual empty destructor. */
RRDAspect::~RRDAspect()
{
}


/** Init RRD aspect.
 * This set the RRD manager to access RRDs.
 * It is guaranteed that this is called for a RRD Thread before start
 * is called (when running regularly inside Fawkes).
 * @param bb RRD to use
 */
void
RRDAspect::init_RRDAspect(RRDManager *rrd_manager)
{
  this->rrd_manager = rrd_manager;
}

} // end namespace fawkes
