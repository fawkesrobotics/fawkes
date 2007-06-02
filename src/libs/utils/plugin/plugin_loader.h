
/***************************************************************************
 *  plugin_loader.h - Loads plugins from .so shared objects
 *
 *  Generated: Wed Aug 23 15:18:13 2006
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __UTILS_PLUGIN_PLUGIN_LOADER_H_
#define __UTILS_PLUGIN_PLUGIN_LOADER_H_

#include <core/plugin.h>
#include <core/exception.h>

class PluginLoaderData;

class PluginLoadException : public Exception
{
 public:
  PluginLoadException(const char *plugin_type, const char *add_msg = NULL);
};

class PluginUnloadException : public Exception
{
 public:
  PluginUnloadException(const char *plugin_type, const char *add_msg = NULL);
};


class PluginLoader {
 public:

  PluginLoader(const char *plugin_base_dir);
  ~PluginLoader();

  Plugin * load(const char *plugin_name);
  void     unload(Plugin *plugin);

  void     request_load(const char *plugin_name);

  bool     finished_load(const char *plugin_name);

  Plugin * finish_deferred_load(const char *plugin_name);

  bool     is_loaded(const char *plugin_name);

 private:
  PluginLoaderData *d;
};


#endif
