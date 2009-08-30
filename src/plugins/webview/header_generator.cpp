
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

#include <cstdio>
#include <cstdlib>

/** Page header template. */
const char *  WebviewHeaderGenerator::PAGE_HEADER =
  "<html>\n"
  " <head>\n"
  "  <title>%s (%s)</title>\n"
  "  <link rel=\"stylesheet\" type=\"text/css\" href=\"/static/webview.css\" />\n"
  " </head>\n"
  " <body>\n"
  "  <div id=\"header\">"
  "<a id=\"logo\" href=\"/\"/><img src=\"/static/webview.png\" alt=\"Fawkes WebView\"/></a>"
  "<hr /></div>\n";

WebviewHeaderGenerator::WebviewHeaderGenerator()
{
}

/** Add navigation entry.
 * @param baseurl baseurl that should be linked for this entry
 * @param name string to print as link name
 */
void
WebviewHeaderGenerator::add_nav_entry(std::string baseurl, std::string name)
{
  __nav_entries[baseurl] = name;
}

/** Remove navigation entry.
 * @param baseurl baseurl whose config entry to remove
 */
void
WebviewHeaderGenerator::remove_nav_entry(std::string baseurl)
{
  __nav_entries.erase(baseurl);
}

std::string
WebviewHeaderGenerator::html_header(std::string &title,
				    std::string &active_baseurl)
{
  fawkes::HostInfo hi;

  std::string rv = "";
  char *s;
  if ( asprintf(&s, PAGE_HEADER, title.c_str(), hi.short_name()) != -1 ) {
    rv = s;
    free(s);
  }

  rv += "  <div id=\"mainnav\" class=\"nav\"><ul>";
  std::map<std::string, std::string>::iterator nei;
  for (nei = __nav_entries.begin(); nei != __nav_entries.end(); ++nei) {
    rv += "<li";
    if ( nei->first == active_baseurl ) {
      rv += " class=\"active\"";
    }
    rv += "><a href=\"" + nei->first + "\">" + nei->second + "</a></li>";
  }
  rv += "</ul></div>";

  return rv;
}
