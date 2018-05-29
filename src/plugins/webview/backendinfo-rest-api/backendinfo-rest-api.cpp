
/***************************************************************************
 *  backendinfo-rest-api.cpp - Backend Info REST API
 *
 *  Created: Mon Apr 09 15:43:14 2018
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

#include "backendinfo-rest-api.h"

#include <webview/rest_api_manager.h>

#include <set>

using namespace fawkes;

/** @class BackendInfoRestApi "skiller-rest-api.h"
 * REST API backend for the image.
 * @author Tim Niemueller
 */

/** Constructor. */
BackendInfoRestApi::BackendInfoRestApi()
	: Thread("BackendInfoRestApi", Thread::OPMODE_WAITFORWAKEUP)
{
}

/** Destructor. */
BackendInfoRestApi::~BackendInfoRestApi()
{
}

void
BackendInfoRestApi::init()
{
	std::set<std::string> configs;
	std::string prefix = "/webview/backends/";

	{
		std::unique_ptr<Configuration::ValueIterator>
			i{config->search(prefix.c_str())};
		while (i->next()) {
			std::string cfg_name = std::string(i->path()).substr(prefix.length());
			cfg_name = cfg_name.substr(0, cfg_name.find("/"));
			configs.insert(cfg_name);
		}
	}
	for (const auto &c : configs) {
		Backend b;
		b.set_kind("Backend");
		b.set_apiVersion(Backend::api_version());
		b.set_id(c);
		b.set_name(config->get_string(prefix + c + "/name"));
		b.set_url(config->get_string(prefix + c + "/url"));

		std::string svc_prefix = prefix + c + "/services/";
		std::unique_ptr<Configuration::ValueIterator>
			i{config->search(svc_prefix.c_str())};
		while (i->next()) {
			std::string svc_name = std::string(i->path()).substr(svc_prefix.length());
			Service s;
			s.set_name(svc_name);
			s.set_url(i->get_string());
			b.addto_services(std::move(s));
		}
		backends_.push_back(std::move(b));
	}

	rest_api_ = new WebviewRestApi("backends", logger);
	rest_api_->add_handler<WebviewRestArray<Backend>>
		(WebRequest::METHOD_GET, "/?",
		 std::bind(&BackendInfoRestApi::cb_list_backends, this));
	webview_rest_api_manager->register_api(rest_api_);
}

void
BackendInfoRestApi::finalize()
{
	webview_rest_api_manager->unregister_api(rest_api_);
	delete rest_api_;
}


void
BackendInfoRestApi::loop()
{
}


WebviewRestArray<Backend>
BackendInfoRestApi::cb_list_backends()
{
	return backends_;
}
