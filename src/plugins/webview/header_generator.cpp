
/***************************************************************************
 *  header_generator.cpp - Generator of page header
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

#include "header_generator.h"

#include <utils/system/hostinfo.h>
#include <webview/nav_manager.h>

#include <cstdio>
#include <cstdlib>

using namespace fawkes;

/** @class WebviewHeaderGenerator "header_generator.h"
 * Webview page header.
 * Custom page header that shows the logo and a navigation bar.
 * @author Tim Niemueller
 */

/** Page header template. */
const char *  WebviewHeaderGenerator::PAGE_HEADER =
  "<html>\n"
  " <head>\n"
  "  <meta http-equiv=\"Content-type\" content=\"text/html; charset=utf-8\" />\n"
  "  <meta http-equiv=\"Content-Language\" content=\"en-us\" />\n"
  "  <title>%s (%s)</title>\n"
  "  <link rel=\"icon\" type=\"image/png\" href=\"/static/images/favicon.png\" />\n"
  "  <link rel=\"stylesheet\" type=\"text/css\" href=\"/static/css/webview.css\" />\n"
  "%s"
  " </head>\n"
  " <body>\n";

/** Constructor.
 * @param nav_manager navigation manager to use to generate the navigation
 */
WebviewHeaderGenerator::WebviewHeaderGenerator(WebNavManager *nav_manager)
{
  nav_manager_ = nav_manager;
}

std::string
WebviewHeaderGenerator::html_header(std::string &title,
				    std::string &active_baseurl,
				    std::string &html_header)
{
  fawkes::HostInfo hi;

  std::string rv = "";
  char *s;
  if ( asprintf(&s, PAGE_HEADER, title.c_str(), hi.short_name(),
		html_header.c_str()) != -1 )
  {
    rv = s;
    free(s);
  }

  rv +=
    "  <div id=\"mainnav\" class=\"nav\"><a id=\"logo\" href=\"/\"/>"
    "<img class=\"navlogo\" src=\"/static/chrome/navlogo.png\" /></a><ul>";
  WebNavManager::NavMap::const_iterator nei;
  const WebNavManager::NavMap &nav_entries(nav_manager_->get_nav_entries());
  for (nei = nav_entries.begin(); nei != nav_entries.end(); ++nei) {
    rv += "<li";
    if ( nei->first == active_baseurl ) {
      rv += " class=\"active\"";
    }
    rv += "><a href=\"" + nei->first + "\">" + nei->second + "</a></li>";
  }
  rv += "</ul></div>";

  return rv;
}
