
/***************************************************************************
 *  navgraph_access_thread.cpp - thread to access navgraph
 *
 *  Created: Mon Aug 20 10:07:23 2018
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "navgraph_access_thread.h"

#include <navgraph/navgraph.h>

/** @class PlexilNavgraphAccessThread "navgraph_access_thread.h"
 * Access to internal navgraph for Plexil.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
PlexilNavgraphAccessThread::PlexilNavgraphAccessThread()
: Thread("PlexilNavgraphAccessThread", Thread::OPMODE_WAITFORWAKEUP)
{
}


/** Destructor. */
PlexilNavgraphAccessThread::~PlexilNavgraphAccessThread()
{
}


/** Get access to navgraph.
 * @return navgraph
 */
fawkes::LockPtr<fawkes::NavGraph>
PlexilNavgraphAccessThread::get_navgraph() const
{
	return navgraph;
}

void
PlexilNavgraphAccessThread::loop()
{
}
