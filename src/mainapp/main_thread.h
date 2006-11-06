
/***************************************************************************
 *  main_thread.h - Fawkes main thread
 *
 *  Created: Thu Nov  2 16:46:37 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __FAWKES_MAIN_THREAD_H_
#define __FAWKES_MAIN_THREAD_H_

#include <core/threading/thread.h>

#include <map>
#include <string>

class BlackBoardInterfaceManager;
class PluginLoader;
class Plugin;
class FawkesThreadManager;

class FawkesMainThread : public Thread
{
 public:
  FawkesMainThread();
  virtual ~FawkesMainThread();

  virtual void loop();

 private:

  BlackBoardInterfaceManager *interface_manager;
  FawkesThreadManager        *thread_manager;
  PluginLoader               *plugin_loader;

  std::map< std::string, Plugin * > plugins;
};

#endif
