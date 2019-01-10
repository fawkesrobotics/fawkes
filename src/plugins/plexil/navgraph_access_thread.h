
/***************************************************************************
 *  navgraph_access_thread.h - thread to get hold of navgraph
 *
 *  Created: Mon Aug 20 10:04:14 2018
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

#ifndef __PLUGINS_PLEXIL_NAVGRAPH_ACCESS_THREAD_H_
#define __PLUGINS_PLEXIL_NAVGRAPH_ACCESS_THREAD_H_

#include <core/threading/thread.h>
#include <navgraph/aspect/navgraph.h>

class PlexilNavgraphAccessThread
: public fawkes::Thread,
  public fawkes::NavGraphAspect
{
 public:
	PlexilNavgraphAccessThread();
	virtual ~PlexilNavgraphAccessThread();

	virtual void loop();

	fawkes::LockPtr<fawkes::NavGraph>   get_navgraph() const;
	
	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

};

#endif
