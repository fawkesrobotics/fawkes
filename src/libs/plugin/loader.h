
/***************************************************************************
 *  loader.h - Loads plugins from .so shared objects
 *
 *  Created: Wed Aug 23 15:18:13 2006
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGIN_LOADER_H_
#define __PLUGIN_LOADER_H_

#include <core/plugin.h>
#include <core/exception.h>

#include <string>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Module;
class Configuration;
class ModuleManager;

class PluginLoadException : public Exception
{
 public:
  PluginLoadException(const char *plugin, const char *message);
  PluginLoadException(const char *plugin, const char *message, Exception &e);
  ~PluginLoadException() throw();

  std::string  plugin_name() const;

 private:
  std::string __plugin_name;
};

class PluginUnloadException : public Exception
{
 public:
  PluginUnloadException(const char *plugin_type, const char *add_msg = NULL);
};


class PluginLoader {
 public:

  PluginLoader(const char *plugin_base_dir, Configuration *config);
  ~PluginLoader();

  Plugin * load(const char *plugin_name);
  void     unload(Plugin *plugin);

  std::string  get_description(const char *plugin_name);

  bool     is_loaded(const char *plugin_name);

  ModuleManager *  get_module_manager() const;

 private:
  Module * open_module(const char *plugin_name);
  std::string  get_string_symbol(const char *plugin_name, const char *symbol_name,
				 const char *section_name = ".fawkes_plugin");
  Plugin * create_instance(const char *plugin_name, Module *module);

 private:
  class Data;

  Data *d_;
  Configuration    *config_;
  std::string       plugin_base_dir_;
};


} // end namespace fawkes

#endif
