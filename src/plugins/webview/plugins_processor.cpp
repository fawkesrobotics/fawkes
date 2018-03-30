
/***************************************************************************
 *  plugins_processor.cpp - Web request processor for plugin info
 *
 *  Created: Thu Feb 12 13:00:37 2009
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

#include "plugins_processor.h"
#include <webview/page_reply.h>
#include <webview/redirect_reply.h>
#include <webview/url_manager.h>

#include <plugin/manager.h>

#include <string>
#include <cstring>
#include <cstdlib>
#include <functional>

using namespace fawkes;

/** @class WebviewPluginsRequestProcessor "plugins_processor.h"
 * Plugins web request processor.
 * Provides access to plugin lists and allows for loading/unloading plugins.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param url_manager URL manager to register with
 * @param manager PluginManager instance
 */
WebviewPluginsRequestProcessor::WebviewPluginsRequestProcessor(fawkes::WebUrlManager *url_manager,
                                                               PluginManager *manager)
{
  manager_     = manager;
  url_manager_ = url_manager;
  url_manager_->add_handler(WebRequest::METHOD_GET, "/plugins/load/{plugin}",
                            std::bind(&WebviewPluginsRequestProcessor::process_load, this,
                                      std::placeholders::_1));
  url_manager_->add_handler(WebRequest::METHOD_GET, "/plugins/unload/{plugin}",
                            std::bind(&WebviewPluginsRequestProcessor::process_unload, this,
                                      std::placeholders::_1));
  url_manager_->add_handler(WebRequest::METHOD_GET, "/plugins/?",
                            std::bind(&WebviewPluginsRequestProcessor::process_overview, this));
}


/** Destructor. */
WebviewPluginsRequestProcessor::~WebviewPluginsRequestProcessor()
{
	url_manager_->remove_handler(WebRequest::METHOD_GET, "/plugins/load/{plugin}");
	url_manager_->remove_handler(WebRequest::METHOD_GET, "/plugins/unload/{plugin}");
	url_manager_->remove_handler(WebRequest::METHOD_GET, "/plugins/?");
}


WebReply *
WebviewPluginsRequestProcessor::process_load(const fawkes::WebRequest *request)
{
	std::string plugin_name = request->path_arg("plugin");
	try {
		manager_->load(plugin_name.c_str());
		return new WebRedirectReply("/plugins/");
	} catch (Exception &e) {
		WebPageReply *r = new WebPageReply("Loading plugin failed");
		r->append_body("<h1>Loading plugin '%s' failed</h1>", plugin_name.c_str());
		*r += "<p>The encountered error was:</p>";
		for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
			*r += std::string(*i) + "<br/>\n";
		}
		r->append_body("<p><a href=\"/plugins/\">Back to overview</a> - "
		               "<a href=\"%s\">Retry</a></p>", request->url().c_str());
		return r;
	}
}

WebReply *
WebviewPluginsRequestProcessor::process_unload(const fawkes::WebRequest *request)
{
	std::string plugin_name = request->path_arg("plugin");
	try {
		manager_->unload(plugin_name.c_str());
		return new WebRedirectReply("/plugins/");
	} catch (Exception &e) {
		WebPageReply *r = new WebPageReply("Unloading plugin failed");
		r->append_body("<h1>Unloading plugin '%s' failed</h1>",
		               plugin_name.c_str());
		*r += "<p>The encountered error was:</p>";
		for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
			*r += std::string(*i) + "<br/>\n";
		}
		r->append_body("<p><a href=\"/plugins/\">Back to overview</a> - "
		               "<a href=\"%s\">Retry</a></p>", request->url().c_str());
		return r;
	}
}

WebReply *
WebviewPluginsRequestProcessor::process_overview()
{
	WebPageReply *r = new WebPageReply("Plugins");
	*r += "<h2>Fawkes Plugins</h2>\n";

	*r +=
		"<style type=\"text/css\">\n"
		"  tr:hover { background-color: #eeeeee; }\n"
		"  :link:hover, :visited:hover { background-color: #bb0000; color: white; }\n"
		"</style>";

	*r += "<table>\n";
	*r += "<tr><th>Name</th><th>Description</th><th>Loaded</th><th>Action</th></tr>\n";

	std::list<std::pair<std::string, std::string> > available_plugins;
	std::list<std::pair<std::string, std::string> >::iterator i;

	available_plugins = manager_->get_available_plugins();

	for (i = available_plugins.begin(); i != available_plugins.end(); ++i) {
		bool is_loaded = manager_->is_loaded(i->first.c_str());

		const char *loaded_color = is_loaded ? "green"  : "red";
		const char *loaded       = is_loaded ? "Yes"    : "No";
		const char *action_link  = is_loaded ? "unload" : "load";

		r->append_body("<tr><td>%s</td><td>%s</td>"
		               "<td><span style=\"color:%s\">%s<span></td>"
		               "<td><a href=\"/plugins/%s/%s\">%s</a></td>\n",
		               i->first.c_str(), i->second.c_str(), loaded_color, loaded,
		               action_link, i->first.c_str(), action_link);
	}

	*r += "</table>\n";

	return r;
}
