
/***************************************************************************
 *  plugin.h - XML-RPC methods related to plugin management
 *
 *  Created: Mon Aug 31 00:50:41 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_XMLRPC_METHODS_PLUGIN_H_
#define __PLUGINS_XMLRPC_METHODS_PLUGIN_H_

#include <xmlrpc-c/registry.hpp>

namespace fawkes {
  class Logger;
  class PluginManager;
}

class XmlRpcPluginMethods {
 public:
  XmlRpcPluginMethods(xmlrpc_c::registry *registry,
		      fawkes::PluginManager *plugin_manager,
		      fawkes::Logger *logger);
  ~XmlRpcPluginMethods();

  class plugin_list : public xmlrpc_c::method {
   public:
    plugin_list(fawkes::PluginManager *plugin_manager);
    virtual ~plugin_list();
    virtual void execute(xmlrpc_c::paramList const& params,
			 xmlrpc_c::value *   const  result);
   private:
    fawkes::PluginManager *__plugin_manager;
  };

  class plugin_load : public xmlrpc_c::method {
   public:
    plugin_load(fawkes::PluginManager *plugin_manager, fawkes::Logger *logger);
    virtual ~plugin_load();
    virtual void execute(xmlrpc_c::paramList const& params,
			 xmlrpc_c::value *   const  result);
   private:
    fawkes::PluginManager *__plugin_manager;
    fawkes::Logger        *__logger;
  };

  class plugin_unload : public xmlrpc_c::method {
   public:
    plugin_unload(fawkes::PluginManager *plugin_manager, fawkes::Logger *logger);
    virtual ~plugin_unload();
    virtual void execute(xmlrpc_c::paramList const& params,
			 xmlrpc_c::value *   const  result);
   private:
    fawkes::PluginManager *__plugin_manager;
    fawkes::Logger        *__logger;
  };

 private:
  xmlrpc_c::registry *__xmlrpc_registry;

  fawkes::PluginManager *__plugin_manager;
  fawkes::Logger        *__logger;
  plugin_list           *__plugin_list;
  plugin_load           *__plugin_load;
  plugin_unload         *__plugin_unload;
};

#endif
