
/***************************************************************************
 *  blackboard_processor.cpp - Web request processor for BlackBoard info
 *
 *  Created: Thu Oct 23 16:10:21 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include "blackboard_processor.h"
#include <webview/page_reply.h>

#include <blackboard/blackboard.h>
#include <interface/interface.h>
#include <interface/field_iterator.h>
#include <interface/interface_info.h>
#include <utils/time/time.h>

#include <string>
#include <cstring>
#include <cstdlib>

using namespace fawkes;

/** @class WebviewBlackBoardRequestProcessor "blackboard_processor.h"
 * BlackBoard web request processor.
 * Provides access to BlackBoard introspection features.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param baseurl Base URL where processor is mounted
 * @param blackboard BlackBoard instance
 */
WebviewBlackBoardRequestProcessor::WebviewBlackBoardRequestProcessor(const char *baseurl,
							     BlackBoard *blackboard)
{
  __baseurl     = strdup(baseurl);
  __baseurl_len = strlen(__baseurl);
  __blackboard  = blackboard;
}


/** Destructor. */
WebviewBlackBoardRequestProcessor::~WebviewBlackBoardRequestProcessor()
{
  free(__baseurl);
  for (__ifi = __interfaces.begin(); __ifi != __interfaces.end(); ++__ifi) {
    __blackboard->close(__ifi->second);
  }
  __interfaces.clear();
}


WebReply *
WebviewBlackBoardRequestProcessor::process_request(const char *url,
					       const char *method,
					       const char *version,
					       const char *upload_data,
					       size_t *upload_data_size,
					       void **session_data)
{
  if ( strncmp(__baseurl, url, __baseurl_len) == 0 ) {
    // It is in our URL prefix range
    std::string subpath = std::string(url).substr(__baseurl_len);

    WebPageReply *r = new WebPageReply("BlackBoard");
    *r += "<h2>BlackBoard interfaces:</h2>\n";

    bool found_some = false;
    InterfaceInfoList *iil = __blackboard->list_all();
    iil->sort();
    for (InterfaceInfoList::iterator i = iil->begin(); i != iil->end(); ++i) {
      if (! found_some) {
        *r += "<table>\n";
        *r += "<tr><th>Interface</th><th>Reader(s)</th><th>Writer</th></tr>\n";
        found_some = true;
      }
      r->append_body("<tr><td><a href=\"%s/view/%s::%s\">%s::%s</a></td><td>%u</td><td style=\"color:%s\">%s</td></tr>\n",
                     __baseurl, i->type(), i->id(), i->type(), i->id(),
                     i->num_readers(), i->has_writer() ? "green" : "red", i->has_writer() ? "yes" : "no");
    }
    delete iil;

    if (found_some) {
      *r += "</table>\n";
    } else {
      *r += "<b>No interfaces found.</b>\n";
    }

    if (subpath.find("/view/") == 0) {
      std::string iuid = subpath.substr(subpath.find_first_not_of("/", std::string("/view/").length()));
      std::string iftype = iuid.substr(0, iuid.find("::"));
      std::string ifname = iuid.substr(iuid.find("::") + 2);


      r->append_body("<a href=\"%s\">Clear detailed</a>\n", __baseurl);

      r->append_body("<h2>Interface: %s</h2>\n", iuid.c_str());
      if (__interfaces.find(iuid) == __interfaces.end()) {
	try {
	  Interface *iface = __blackboard->open_for_reading(iftype.c_str(), ifname.c_str());
	  __interfaces[iuid] = iface;
	} catch (Exception &e) {
	  r->append_body("Failed to open interface: %s\n", e.what());
	}
      }
      if (__interfaces.find(iuid) != __interfaces.end()) {
	Interface *iface = __interfaces[iuid];
	iface->read();

	r->append_body("<table>\n"
		       " <tr><td><b>Type:</b></td><td>%s</td></tr>\n"
		       " <tr><td><b>ID:</b></td><td>%s</td></tr>\n"
		       " <tr><td><b>Has writer?:</b></td><td>%s</td></tr>\n"
		       " <tr><td><b>Num readers:</b></td><td>%u</td></tr>\n"
		       " <tr><td><b>Serial:</b></td><td>%u</td></tr>\n"
		       " <tr><td><b>Data size:</b></td><td>%u</td></tr>\n"
		       " <tr><td><b>Hash:</b></td><td>%s</td></tr>\n"
		       " <tr><td><b>Data changed:</b></td>"
		       "<td>%s (last at %s)</td></tr>\n"
		       "</table>\n",
		       iface->type(), iface->id(), iface->has_writer() ? "yes" : "no",
		       iface->num_readers(), iface->serial(),
		       iface->datasize(), iface->hash_printable(),
		       iface->changed() ? "yes" : "no", iface->timestamp()->str());

	r->append_body("<table>\n"
		       " <tr>\n"
		       "  <th>Name</th><th>Type</th><th>Value</th>\n"
		       " </tr>\n");
	for (InterfaceFieldIterator fi = iface->fields(); fi != iface->fields_end(); ++fi) {
	  bool is_string = (fi.get_type() == IFT_STRING);
	  *r += " <tr>\n";
	  if ( fi.get_length() > 1 ) {
	    r->append_body("  <td>%s</td><td>%s [%zu]</td><td>%s%s%s</td>\n",
			   fi.get_name(), fi.get_typename(),
			   fi.get_length(), is_string ? "<pre>" : "",
			   fi.get_value_string(), is_string ? "</pre>" : "");
	  } else {
	    r->append_body("  <td>%s</td><td>%s</td><td>%s%s%s</td>\n",
			   fi.get_name(), fi.get_typename(), is_string ? "<pre>" : "",
			   fi.get_value_string(), is_string ? "</pre>" : "");
	  }
	  *r += " </tr>\n";
	}
	r->append_body("</table>\n");
      }
    }

    return r;
  } else {
    return NULL;
  }
}
