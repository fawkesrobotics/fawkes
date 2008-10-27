
/***************************************************************************
 *  load_thread.h - Plugin load thread
 *
 *  Created: Thu May 31 12:04:18 2007
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

#include <plugin/loader/load_thread.h>

#include <utils/system/dynamic_module/module_manager.h>
#include <utils/system/dynamic_module/module.h>

#include <string>

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <cstdio>
#include <cstdlib>

namespace fawkes {

/** @class PluginLoadThread <plugin/loader/load_thread.h>
 * Plugin load thread.
 * This thread is used internally by the plugin loader for asynchronous
 * load operations.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param mm module manager to use to load the module containing the plugin
 * @param plugin_name name of the plugin to load
 * @param config Fawkes configuration
 */
PluginLoadThread::PluginLoadThread(ModuleManager *mm, const char *plugin_name,
				   Configuration *config)
  : Thread((std::string("PluginLoadThread::") + std::string(plugin_name)).c_str()),
    ple(plugin_name)
{
  _mm = mm;
  _finished = false;
  _module = NULL;
  _plugin = NULL;
  _config = config;

  // This is dependent on the system architecture!
  if ( asprintf(&_module_name, "%s.%s", plugin_name, _mm->getModuleFileExtension()) == -1 ) {
    ple.append("Could not allocate module_name buffer");
    _module_name = NULL;
  }
}


/** Destructor. */
PluginLoadThread::~PluginLoadThread()
{
  if ( _module_name != NULL ) {
    free(_module_name);
  }
}


/** Check if load operation is finished.
 * @return true if load operation is finished, false otherwise.
 */
bool
PluginLoadThread::finished()
{
  return _finished;
}


/** Get the loaded plugin.
 * @return loaded plugin
 * @exception PluginLoadException thrown if operation is not complete.
 */
Plugin *
PluginLoadThread::plugin()
{
  if ( ! _finished ) {
    throw PluginLoadException("PluginLoadThread: load operation not yet complete");
  } else if (_plugin == NULL) {
    throw ple;
  } else {
    return _plugin;
  }
}


/** Get the module of the loaded plugin.
 * @return module of loaded plugin
 * @exception PluginLoadException thrown if operation is not complete.
 */
Module *
PluginLoadThread::module()
{
  if ( ! _finished ) {
    throw PluginLoadException("PluginLoadThread: load operation not yet complete");
  } else if (_module == NULL) {
    throw ple;
  } else {
    return _module;
  }
}


/** Load the plugin. */
void
PluginLoadThread::load()
{
  if ( _module_name == NULL )  return;

  try {
    _module = _mm->openModule(_module_name);
  } catch (ModuleOpenException &e) {
    ple.append("PluginLoader failed to open module %s", _module_name);
    ple.append(e);
    return;
  }

  if ( _module == NULL ) {
    // we could NOT open the plugin module
    ple.append("Could not open plugin module '%s'", _module_name);
    return;
  }

  if ( ! _module->hasSymbol("plugin_factory") ) {
    ple.append("Symbol 'plugin_factory' not found");
    return;
  }

  PluginFactoryFunc pff = (PluginFactoryFunc)_module->getSymbol("plugin_factory");

  try {
    Plugin *p = pff(_config);
    if ( p == NULL ) {
      ple.append("Plugin from module '%s' could not be instantiated", _module_name);
    } else {
      _plugin = p;
      _plugin->set_name(_module_name);
    }
  } catch (Exception &e) {
    ple.append("Plugin instantiation caused exception, trace follows.");
    ple.append(e);
  }

}

/** Load blocking.
 * Instead of doing the loading in the background this will load the plugin and block
 * until the plugin is loaded or an error occured.
 */
void
PluginLoadThread::load_blocking()
{
  load();
  _finished = true;
}


/** Thread loop. */
void
PluginLoadThread::loop()
{
  load();
  _finished = true;
  exit();
}


} // end namespace fawkes
