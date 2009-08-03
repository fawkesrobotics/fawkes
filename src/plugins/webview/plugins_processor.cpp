
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
#include "page_reply.h"
#include "redirect_reply.h"

#include <plugin/manager.h>

#include <string>
#include <cstring>
#include <cstdlib>

using namespace fawkes;

/** @class WebPluginsRequestProcessor "plugins_processor.h"
 * Plugins web request processor.
 * Provides access to plugin lists and allows for loading/unloading plugins.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param baseurl Base URL where processor is mounted
 * @param manager PluginManager instance
 */
WebPluginsRequestProcessor::WebPluginsRequestProcessor(const char *baseurl,
						       PluginManager *manager)
{
  __baseurl     = strdup(baseurl);
  __baseurl_len = strlen(__baseurl);
  __manager     = manager;
}


/** Destructor. */
WebPluginsRequestProcessor::~WebPluginsRequestProcessor()
{
  free(__baseurl);
}


WebReply *
WebPluginsRequestProcessor::process_request(const char *url,
					       const char *method,
					       const char *version,
					       const char *upload_data,
					       size_t *upload_data_size,
					       void **session_data)
{
  if ( strncmp(__baseurl, url, __baseurl_len) == 0 ) {
    // It is in our URL prefix range
    std::string subpath = std::string(url).substr(__baseurl_len);

    if (subpath.find("/load/") == 0) {
      std::string plugin_name = subpath.substr(std::string("/load/").length());
      __manager->load(plugin_name.c_str());
      return new WebRedirectReply(__baseurl);
    } else if (subpath.find("/unload/") == 0) {
      std::string plugin_name = subpath.substr(std::string("/unload/").length());
      __manager->unload(plugin_name.c_str());
      return new WebRedirectReply(__baseurl);
    } else {
      WebPageReply *r = new WebPageReply("BlackBoard");
      *r += "<h2>Fawkes Plugins</h2>\n";

      *r += "<table>\n";
      *r += "<tr><th>Name</th><th>Description</th><th>Loaded</th><th>Action</th></tr>\n";

      std::list<std::pair<std::string, std::string> > available_plugins;
      std::list<std::pair<std::string, std::string> >::iterator i;

      available_plugins = __manager->get_available_plugins();

      for (i = available_plugins.begin(); i != available_plugins.end(); ++i) {
	bool is_loaded = __manager->is_loaded(i->first.c_str());

	const char *loaded_color = is_loaded ? "green"  : "red";
	const char *loaded       = is_loaded ? "Yes"    : "No";
	const char *action_link  = is_loaded ? "unload" : "load";

	r->append_body("<tr><td>%s</td><td>%s</td>"
		       "<td><span style=\"color:%s\">%s<span></td>"
		       "<td><a href=\"%s/%s/%s\">%s</a></td>\n",
		       i->first.c_str(), i->second.c_str(), loaded_color, loaded,
		       __baseurl, action_link, i->first.c_str(), action_link);
      }

      *r += "</table>\n";

      return r;
    }
  } else {
    return NULL;
  }
}
