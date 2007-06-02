
/***************************************************************************
 *  load_thread.cpp - Plugin load thread
 *
 *  Created: Thu May 31 15:07:18 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __UTILS_PLUGIN_LOAD_THREAD_H_
#define __UTILS_PLUGIN_LOAD_THREAD_H_

#include <core/threading/thread.h>
#include <utils/plugin/plugin_loader.h>

class ModuleManager;
class Plugin;
class Module;

class PluginLoadThread : public Thread
{
 public:
  PluginLoadThread(ModuleManager *mm, const char *plugin_name);
  virtual ~PluginLoadThread();

  Plugin *  plugin();
  Module *  module();
  void      load_blocking();
  bool      finished();

  virtual void loop();

 private:
  void      load();

  char          *module_name;
  bool           _finished;
  ModuleManager *mm;
  Plugin        *_plugin;
  Module        *_module;
  PluginLoadException ple;
};

#endif
