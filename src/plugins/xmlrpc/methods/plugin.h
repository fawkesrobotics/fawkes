
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

#ifndef _PLUGINS_XMLRPC_METHODS_PLUGIN_H_
#define _PLUGINS_XMLRPC_METHODS_PLUGIN_H_

#include <xmlrpc-c/registry.hpp>

namespace fawkes {
class Logger;
class PluginManager;
} // namespace fawkes

class XmlRpcPluginMethods
{
public:
	XmlRpcPluginMethods(std::shared_ptr<xmlrpc_c::registry> registry,
	                    fawkes::PluginManager *             plugin_manager,
	                    fawkes::Logger *                    logger);
	~XmlRpcPluginMethods();

	class plugin_list : public xmlrpc_c::method
	{
	public:
		plugin_list(fawkes::PluginManager *plugin_manager);
		virtual ~plugin_list();
		virtual void execute(xmlrpc_c::paramList const &params, xmlrpc_c::value *const result);

	private:
		fawkes::PluginManager *plugin_manager_;
	};

	class plugin_load : public xmlrpc_c::method
	{
	public:
		plugin_load(fawkes::PluginManager *plugin_manager, fawkes::Logger *logger);
		virtual ~plugin_load();
		virtual void execute(xmlrpc_c::paramList const &params, xmlrpc_c::value *const result);

	private:
		fawkes::PluginManager *plugin_manager_;
		fawkes::Logger *       logger_;
	};

	class plugin_unload : public xmlrpc_c::method
	{
	public:
		plugin_unload(fawkes::PluginManager *plugin_manager, fawkes::Logger *logger);
		virtual ~plugin_unload();
		virtual void execute(xmlrpc_c::paramList const &params, xmlrpc_c::value *const result);

	private:
		fawkes::PluginManager *plugin_manager_;
		fawkes::Logger *       logger_;
	};

private:
	std::shared_ptr<xmlrpc_c::registry> xmlrpc_registry_;

	fawkes::PluginManager *        plugin_manager_;
	fawkes::Logger *               logger_;
	std::unique_ptr<plugin_list>   plugin_list_;
	std::unique_ptr<plugin_load>   plugin_load_;
	std::unique_ptr<plugin_unload> plugin_unload_;
};

#endif
