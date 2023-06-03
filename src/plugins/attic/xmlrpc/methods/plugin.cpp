
/***************************************************************************
 *  plugin.cpp - XML-RPC methods related to plugin management
 *
 *  Created: Mon Aug 31 00:56:55 2009
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

#include "plugin.h"

#include <logging/logger.h>
#include <plugin/manager.h>

#include <algorithm>
#include <map>
#include <string>
#include <vector>
#include <xmlrpc-c/girerr.hpp>

/** @class XmlRpcPluginMethods "plugin.h"
 * Wrapper class for plugin related XML-RPC methods.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param registry XML registry, methods will be automatically registered
 * @param plugin_manager plugin manager used for listing, loading and unloading plugins
 * @param logger logger to output informational and error messages
 */
XmlRpcPluginMethods::XmlRpcPluginMethods(std::shared_ptr<xmlrpc_c::registry> registry,
                                         fawkes::PluginManager              *plugin_manager,
                                         fawkes::Logger                     *logger)
{
	xmlrpc_registry_ = registry;
	plugin_manager_  = plugin_manager;
	logger_          = logger;
	plugin_list_.reset(new plugin_list(plugin_manager));
	plugin_load_.reset(new plugin_load(plugin_manager, logger));
	plugin_unload_.reset(new plugin_unload(plugin_manager, logger));
	xmlrpc_registry_->addMethod("plugin.list", &*plugin_list_);
	xmlrpc_registry_->addMethod("plugin.load", &*plugin_load_);
	xmlrpc_registry_->addMethod("plugin.unload", &*plugin_unload_);
}

/** Destructor. */
XmlRpcPluginMethods::~XmlRpcPluginMethods()
{
	plugin_list_.reset();
	plugin_load_.reset();
	plugin_unload_.reset();
}

/** @class XmlRpcPluginMethods::plugin_list "plugin.h"
 * Plugin list XML-RPC method.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param plugin_manager plugin manager to query for plugin listings.
 */
XmlRpcPluginMethods::plugin_list::plugin_list(fawkes::PluginManager *plugin_manager)
{
	_signature = "A:";
	_help      = "Returns array of plugins. Each entry is a struct consisting of the "
	             "entries name, desc, and loaded.";

	plugin_manager_ = plugin_manager;
}

/** Virtual empty destructor. */
XmlRpcPluginMethods::plugin_list::~plugin_list()
{
}

/** Execute method.
 * @param params parameters
 * @param result result value
 */
void
XmlRpcPluginMethods::plugin_list::execute(xmlrpc_c::paramList const &params,
                                          xmlrpc_c::value *const     result)
{
	std::list<std::pair<std::string, std::string>> available_plugins;
	std::list<std::string>                         loadedp;
	available_plugins = plugin_manager_->get_available_plugins();
	loadedp           = plugin_manager_->get_loaded_plugins();

	loadedp.sort();

	std::vector<xmlrpc_c::value> array;

	std::list<std::pair<std::string, std::string>>::iterator i;
	for (i = available_plugins.begin(); i != available_plugins.end(); ++i) {
		std::map<std::string, xmlrpc_c::value> elem;
		elem.insert(std::make_pair("name", xmlrpc_c::value_string(i->first)));
		elem.insert(std::make_pair("desc", xmlrpc_c::value_string(i->second)));
		bool loaded = std::binary_search(loadedp.begin(), loadedp.end(), i->first);
		elem.insert(std::make_pair("loaded", xmlrpc_c::value_boolean(loaded)));
		array.push_back(xmlrpc_c::value_struct(elem));
	}

	*result = xmlrpc_c::value_array(array);
}

/** @class XmlRpcPluginMethods::plugin_load "plugin.h"
 * XML-RPC method to load a plugin.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param plugin_manager plugin manager to query for plugin listings.
 * @param logger logger to report problems
 */
XmlRpcPluginMethods::plugin_load::plugin_load(fawkes::PluginManager *plugin_manager,
                                              fawkes::Logger        *logger)
{
	_signature = "b:s";
	_help      = "Load plugin specified as argument, returns true on success, false otherwise.";

	plugin_manager_ = plugin_manager;
	logger_         = logger;
}

/** Virtual empty destructor. */
XmlRpcPluginMethods::plugin_load::~plugin_load()
{
}

/** Execute method.
 * @param params parameters
 * @param result result value
 */
void
XmlRpcPluginMethods::plugin_load::execute(xmlrpc_c::paramList const &params,
                                          xmlrpc_c::value *const     result)
{
	try {
		std::string plugin_name = params.getString(0);
		plugin_manager_->load(plugin_name.c_str());
	} catch (girerr::error &e) {
		throw xmlrpc_c::fault(e.what(), xmlrpc_c::fault::CODE_UNSPECIFIED);
	} catch (fawkes::Exception &e) {
		logger_->log_warn("XML-RPC plugin.load", e);
		*result = xmlrpc_c::value_boolean(false);
	}

	*result = xmlrpc_c::value_boolean(true);
}

/** @class XmlRpcPluginMethods::plugin_unload "plugin.h"
 * XML-RPC method to unload a plugin.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param plugin_manager plugin manager to query for plugin listings.
 * @param logger logger to report problems
 */
XmlRpcPluginMethods::plugin_unload::plugin_unload(fawkes::PluginManager *plugin_manager,
                                                  fawkes::Logger        *logger)
{
	_signature = "b:s";
	_help      = "Unload plugin specified as argument, returns true on success, false otherwise.";

	plugin_manager_ = plugin_manager;
	logger_         = logger;
}

/** Virtual empty destructor. */
XmlRpcPluginMethods::plugin_unload::~plugin_unload()
{
}

/** Execute method.
 * @param params parameters
 * @param result result value
 */
void
XmlRpcPluginMethods::plugin_unload::execute(xmlrpc_c::paramList const &params,
                                            xmlrpc_c::value *const     result)
{
	try {
		std::string plugin_name = params.getString(0);
		plugin_manager_->unload(plugin_name.c_str());
	} catch (girerr::error &e) {
		throw xmlrpc_c::fault(e.what(), xmlrpc_c::fault::CODE_UNSPECIFIED);
	} catch (fawkes::Exception &e) {
		logger_->log_warn("XML-RPC plugin.unload", e);
		*result = xmlrpc_c::value_boolean(false);
	}

	*result = xmlrpc_c::value_boolean(true);
}
