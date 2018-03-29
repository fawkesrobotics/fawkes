
/***************************************************************************
 *  footer_generator.cpp - Generator of page footer
 *
 *  Created: Sun Aug 30 14:40:26 2009
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

#include "footer_generator.h"

#include <core/version.h>
#include <utils/misc/string_conversions.h>

/** @class WebviewFooterGenerator "footer_generator.h"
 * Webview page footer.
 * Custom page header that shows other webview instances found on the net
 * via mDNS-SD.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param service_browser service browser used to add links to other Webview
 * instances.
 */
WebviewFooterGenerator::WebviewFooterGenerator(WebviewServiceBrowseHandler *service_browser)
{
  service_browser_ = service_browser;
}


std::string
WebviewFooterGenerator::html_footer()
{
  std::string f = std::string("\n  <div id=\"footer\">\n")
    + "    <hr />\n";

  f += "    <div id=\"version\"><a href=\"http://www.fawkesrobotics.org\" "
    "rel=\"external\">Fawkes ";
  f += FAWKES_VERSION_STRING;
  f += "</a></div>\n";
  WebviewServiceBrowseHandler::ServiceList sl = service_browser_->service_list();
  if (! sl.empty()) {
    f += "    <div class=\"instances\"><ul>";
    WebviewServiceBrowseHandler::ServiceList &sl = service_browser_->service_list();
    WebviewServiceBrowseHandler::ServiceList::iterator i;
    for (i = sl.begin(); i != sl.end(); ++i) {
      std::string short_host = i->second->host();
      std::string::size_type s = short_host.find(".");
      if (s != std::string::npos)  short_host = short_host.substr(0, s);

      f += std::string("<li><a href=\"http://") + i->second->host() + ":"
	+ fawkes::StringConversions::to_string(i->second->port()) + "/\""
	+ " title=\"" + i->first + "\">"
	+ short_host + "</a></li>";
    }
    f += "</ul></div>\n";
  }
  f += "  </div>";
  f += "\n </body>\n";
  f += "</html>\n";

  return f;
}
