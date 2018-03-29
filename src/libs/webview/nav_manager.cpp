
/***************************************************************************
 *  url_manager.cpp - Web URL manager
 *
 *  Created: Thu Nov 25 21:56:19 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
 *
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

#include <webview/nav_manager.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <core/exception.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


/** @class WebNavManager <webview/nav_manager.h>
 * Manage visible navigation entries.
 * This class maintains a map from URLs to names, which are to be added to
 * the page navigation.
 * @author Tim Niemueller
 */

/** Constructor. */
WebNavManager::WebNavManager()
{
  mutex_ = new Mutex();
}


/** Destructor. */
WebNavManager::~WebNavManager()
{
  delete mutex_;
}


/** Add a navigation entry.
 * @param baseurl URL for the navigation target
 * @param name name to display to the user
 * @exception Exception thrown if navigation entry already exists
 */
void
WebNavManager::add_nav_entry(std::string baseurl, std::string name)
{
  MutexLocker lock(mutex_);
  if (nav_entries_.find(baseurl) != nav_entries_.end()) {
    throw Exception("Navigation entry for %s has already been added",
		    baseurl.c_str());
  }
  nav_entries_[baseurl] = name;
}


/** Remove a navigation entry.
 * @param baseurl URL for which to remove the navigation entry.
 */
void
WebNavManager::remove_nav_entry(std::string baseurl)
{
  MutexLocker lock(mutex_);
  nav_entries_.erase(baseurl);
}

} // end namespace fawkes
