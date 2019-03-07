
/***************************************************************************
 *  plugin_director_thread.h - CEDAR plugin manager access
 *
 *  Created: Fri Dec 13 18:21:18 2013
 *  Copyright  2006-2013  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_CEDAR_PLUGIN_DIRECTOR_THREAD_H_
#define __PLUGINS_CEDAR_PLUGIN_DIRECTOR_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/plugin_director.h>

#include <list>
#include <string>

class CedarPluginDirectorThread
: public fawkes::Thread,
  public fawkes::PluginDirectorAspect
{
 /** Cedar thread can access private API. */
 friend class CedarThread;
 public:
  CedarPluginDirectorThread();
  virtual ~CedarPluginDirectorThread();

  virtual void loop();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  std::list<std::string> get_loaded_plugins()
  { return plugin_manager->get_loaded_plugins(); }

  std::list<std::pair<std::string, std::string> >  get_available_plugins()
  { return plugin_manager->get_available_plugins(); }

};

#endif
