
/***************************************************************************
 *  manager.h - Fawkes plugin manager
 *
 *  Created: Wed Nov 15 23:28:01 2006
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __PLUGIN_MANAGER_H_
#define __PLUGIN_MANAGER_H_

#include <netcomm/fawkes/handler.h>
#include <core/utils/lock_list.h>
#include <core/utils/lock_map.h>
#include <config/change_handler.h>
#include <utils/system/fam.h>

#include <string>
#include <utility>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ThreadCollector;
class Plugin;
class PluginLoader;
class Mutex;
class PluginListMessage;
class Configuration;
#ifdef HAVE_INOTIFY
class FamThread;
#endif
class PluginManagerListener;

class PluginManager
: public fawkes::ConfigurationChangeHandler,
  public FamListener
{
 public:
  PluginManager(ThreadCollector *thread_collector,
		Configuration *config,
		const char *meta_plugin_prefix);
  ~PluginManager();

  // for ConfigurationChangeHandler
  virtual void config_tag_changed(const char *new_tag);
  virtual void config_value_changed(const char *path, bool is_default, int value);
  virtual void config_value_changed(const char *path, bool is_default, unsigned int value);
  virtual void config_value_changed(const char *path, bool is_default, float value);
  virtual void config_value_changed(const char *path, bool is_default, bool value);
  virtual void config_value_changed(const char *path, bool is_default, const char *value);
  virtual void config_comment_changed(const char *path, bool is_default, const char *comment);
  virtual void config_value_erased(const char *path, bool is_default);

  // for FamListener
  virtual void fam_event(const char *filename, unsigned int mask);

  void load(const char *plugin_list);
  void unload(const char *plugin_name);

  bool is_loaded(const char *plugin_name);

  std::list<std::string>                           get_loaded_plugins();
  std::list<std::pair<std::string, std::string> >  get_available_plugins();

  void add_listener(PluginManagerListener *listener);
  void remove_listener(PluginManagerListener *listener);

 private:
  void init_pinfo_cache();
  void notify_loaded(const char *plugin_name);
  void notify_unloaded(const char *plugin_name);

  std::list<std::string>  parse_plugin_list(const char *plugin_type_list);

 private:
  ThreadCollector   *thread_collector;
  PluginLoader      *plugin_loader;

  LockMap< std::string, Plugin * > plugins;
  LockMap< std::string, Plugin * >::iterator pit;
  LockMap< std::string, Plugin * >::reverse_iterator rpit;

  LockMap< std::string, std::string > __meta_plugins;
  LockMap< std::string, std::string >::iterator __mpit;

  unsigned int next_plugin_id;
  std::map< std::string, unsigned int > plugin_ids;

  LockList<std::pair<std::string, std::string> > __pinfo_cache;

  LockList<PluginManagerListener *>           __listeners;
  LockList<PluginManagerListener *>::iterator __lit;

  Configuration *__config;
  std::string __meta_plugin_prefix;

#ifdef HAVE_INOTIFY
  FamThread *__fam_thread;
#endif
};

} // end namespace fawkes

#endif
