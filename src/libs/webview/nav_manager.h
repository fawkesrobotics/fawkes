
/***************************************************************************
 *  nav_manager.h - Web Navigation manager
 *
 *  Created: Tue Dec 21 01:28:50 2010
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

#ifndef __LIBS_WEBVIEW_NAV_MANAGER_H_
#define __LIBS_WEBVIEW_NAV_MANAGER_H_

#include <map>
#include <string>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Mutex;

class WebNavManager
{
 public:
  /** Navigation map type, mapping URLs to labels. */
  typedef std::map<std::string, std::string> NavMap;

  WebNavManager();
  ~WebNavManager();

  void add_nav_entry(std::string baseurl, std::string name);
  void remove_nav_entry(std::string baseurl);

  /** Get navigation entries. @return navigation entries map. */
  const NavMap & get_nav_entries() const { return nav_entries_; }
  /** Get mutex for navigation entries. @return mutex for navigation entries. */
  Mutex *  mutex() { return mutex_; }

 private:
  Mutex                                        *mutex_;
  NavMap nav_entries_;
};

} // end namespace fawkes

#endif
