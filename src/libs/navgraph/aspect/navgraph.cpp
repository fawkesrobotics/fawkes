
/***************************************************************************
 *  navgraph.cpp - NavGraph aspect for Fawkes
 *
 *  Created: Tue Oct 08 18:13:41 2013
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
 *             2014       Sebastian Reuter
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

#include <navgraph/aspect/navgraph.h>
#include <navgraph/navgraph.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NavGraphAspect <navgraph/aspect/navgraph.h>
 * Thread aspect to access NavGraph.
 * Give this aspect to your thread to gain access to NavGraph. This will
 * setup the navgraph member with the globally shared instance of
 * NavGraph.
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */

/** @var fawkes::LockPtr<fawkes::NavGraph>  NavGraphAspect::navgraph
 * NavGraph instance shared in framework.
 */

/** Constructor. */
NavGraphAspect::NavGraphAspect()
{
  add_aspect("NavGraphAspect");
}


/** Virtual empty destructor. */
NavGraphAspect::~NavGraphAspect()
{
}

} // end namespace fawkes
