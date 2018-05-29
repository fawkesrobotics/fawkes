
/***************************************************************************
 *  plugin-rest-api.cpp -  Plugin REST API
 *
 *  Created: Tue Apr 10 17:10:43 2018
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
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

#include "plugin-rest-api.h"

#include <webview/rest_api_manager.h>

#include "model/Plugin.h"

using namespace fawkes;

/** @class PluginRestApi "skiller-rest-api.h"
 * REST API backend for plugins.
 * @author Tim Niemueller
 */

/** Constructor. */
PluginRestApi::PluginRestApi()
	: Thread("PluginRestApi", Thread::OPMODE_WAITFORWAKEUP)
{
}

/** Destructor. */
PluginRestApi::~PluginRestApi()
{
}

void
PluginRestApi::init()
{
	rest_api_ = new WebviewRestApi("plugins", logger);
	rest_api_->add_handler<WebviewRestArray<::Plugin>>
		(WebRequest::METHOD_GET, "/?",
		 std::bind(&PluginRestApi::cb_list_plugins, this));
	rest_api_->add_handler<PluginOpResponse, PluginOpRequest>
		(WebRequest::METHOD_PUT, "/{name}",
		 std::bind(&PluginRestApi::cb_set_plugin_state, this,
		           std::placeholders::_1, std::placeholders::_2));
	webview_rest_api_manager->register_api(rest_api_);
}

void
PluginRestApi::finalize()
{
	webview_rest_api_manager->unregister_api(rest_api_);
	delete rest_api_;
}


void
PluginRestApi::loop()
{
}

WebviewRestArray<::Plugin>
PluginRestApi::cb_list_plugins()
{
	WebviewRestArray<::Plugin> rv;

	auto available_plugins = plugin_manager->get_available_plugins();
	for (const auto &i : available_plugins) {
		const std::string& name = i.first;
		const std::string& description = i.second;
		bool is_loaded = plugin_manager->is_loaded(name.c_str());
		bool is_meta = plugin_manager->is_meta_plugin(name);
		::Plugin p;
		p.set_kind("Plugin");
		p.set_apiVersion(::Plugin::api_version());
		p.set_name(name);
		p.set_description(description);
		p.set_is_meta(is_meta);
		if (is_meta) {
			auto plugin_list = plugin_manager->get_meta_plugin_children(name);
			std::vector<std::string>
				v{std::make_move_iterator(std::begin(plugin_list)), 
					std::make_move_iterator(std::end(plugin_list))};
			p.set_meta_children(std::move(v));
		}
		p.set_is_loaded(is_loaded);
		rv.push_back(std::move(p));
	}

	return rv;
}

PluginOpResponse
PluginRestApi::cb_set_plugin_state(PluginOpRequest request, WebviewRestParams &params)
{
	std::string plugin = params.path_arg("name");

	PluginOpResponse response;
	response.set_kind("PluginOpResponse");
	response.set_apiVersion(PluginOpResponse::api_version());
	response.set_name(plugin);
	
	auto des_state = request.desired_state();
	if (! des_state) {
		response.set_state("ERROR");
		response.set_message("Request is missing required field 'desired_state'");
		throw WebviewRestException(WebReply::HTTP_BAD_REQUEST, response,
		                           params.has_query_arg("pretty"));
	}

	if (*des_state == "LOADED") {
		try {
			plugin_manager->load(plugin);
			response.set_state("LOADED");
			return response;
		} catch (Exception &e) {
			logger->log_error(name(), e);
			response.set_state("ERROR");
			response.set_message(e.what_no_backtrace());
			throw WebviewRestException(WebReply::HTTP_INTERNAL_SERVER_ERROR, response,
			                           params.has_query_arg("pretty"));
		}
	} else if (*des_state == "AVAILABLE" || *des_state == "UNLOADED") {
		try {
			plugin_manager->unload(plugin);
			response.set_state(*des_state);
			return response;
		} catch (Exception &e) {
			logger->log_error(name(), e);
			response.set_state("ERROR");
			response.set_message(e.what_no_backtrace());
			throw WebviewRestException(WebReply::HTTP_INTERNAL_SERVER_ERROR, response,
			                           params.has_query_arg("pretty"));
		}		
	} else {
		response.set_state("ERROR");
		response.set_message("Unknown state requested");
		throw WebviewRestException(WebReply::HTTP_BAD_REQUEST, response,
		                           params.has_query_arg("pretty"));
	}
}
