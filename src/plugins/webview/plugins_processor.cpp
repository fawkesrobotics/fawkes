
/***************************************************************************
 *  plugins_processor.cpp - Web request processor for plugin info
 *
 *  Created: Thu Feb 12 13:00:37 2009
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

#include "plugins_processor.h"
#include <webview/page_reply.h>
#include <webview/redirect_reply.h>

#include <plugin/manager.h>

#include <string>
#include <cstring>
#include <cstdlib>

using namespace fawkes;

/** @class WebviewPluginsRequestProcessor "plugins_processor.h"
 * Plugins web request processor.
 * Provides access to plugin lists and allows for loading/unloading plugins.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param baseurl Base URL where processor is mounted
 * @param manager PluginManager instance
 */
WebviewPluginsRequestProcessor::WebviewPluginsRequestProcessor(const char *baseurl,
							       PluginManager *manager)
{
  baseurl_     = strdup(baseurl);
  baseurl_len_ = strlen(baseurl_);
  manager_     = manager;
}


/** Destructor. */
WebviewPluginsRequestProcessor::~WebviewPluginsRequestProcessor()
{
  free(baseurl_);
}


WebReply *
WebviewPluginsRequestProcessor::process_request(const fawkes::WebRequest *request)
{
  if ( strncmp(baseurl_, request->url().c_str(), baseurl_len_) == 0 ) {
    // It is in our URL prefix range
    std::string subpath = request->url().substr(baseurl_len_);

    if (subpath.find("/load/") == 0) {
      std::string plugin_name = subpath.substr(std::string("/load/").length());
      try {
	manager_->load(plugin_name.c_str());
	return new WebRedirectReply(baseurl_);
      } catch (Exception &e) {
	WebPageReply *r = new WebPageReply("Loading plugin failed");
	r->append_body("<h1>Loading plugin '%s' failed</h1>", plugin_name.c_str());
	*r += "<p>The encountered error was:</p>";
	for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
	  *r += std::string(*i) + "<br/>\n";
	}
	r->append_body("<p><a href=\"%s\">Back to overview</a> - "
		       "<a href=\"%s\">Retry</a></p>", baseurl_, request->url().c_str());
	return r;
      }
    } else if (subpath.find("/unload/") == 0) {
      std::string plugin_name = subpath.substr(std::string("/unload/").length());
      try {
	manager_->unload(plugin_name.c_str());
	return new WebRedirectReply(baseurl_);
      } catch (Exception &e) {
	WebPageReply *r = new WebPageReply("Unloading plugin failed");
	r->append_body("<h1>Unloading plugin '%s' failed</h1>",
		       plugin_name.c_str());
	*r += "<p>The encountered error was:</p>";
	for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
	  *r += std::string(*i) + "<br/>\n";
	}
	r->append_body("<p><a href=\"%s\">Back to overview</a> - "
		       "<a href=\"%s\">Retry</a></p>", baseurl_, request->url().c_str());
	return r;
      }
    } else {
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
		       "<td><a href=\"%s/%s/%s\">%s</a></td>\n",
		       i->first.c_str(), i->second.c_str(), loaded_color, loaded,
		       baseurl_, action_link, i->first.c_str(), action_link);
      }

      *r += "</table>\n";

      return r;
    }
  } else {
    return NULL;
  }
}
