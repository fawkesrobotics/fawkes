
/***************************************************************************
 *  page_reply.h - Web request reply for a normal page
 *
 *  Created: Thu Oct 23 16:13:48 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include "page_reply.h"

#include <cstdlib>
#include <cstring>
#include <cstdio>

/** @class WebPageReply "page_reply.h"
 * Basic page reply.
 * This reply adds header and footer as appropriate to form a HTML document
 * with logo and navigation.
 * @author Tim Niemueller
 */

/** Page header template. */
const char *  WebPageReply::PAGE_HEADER =
  "<html>\n"
  " <head>\n"
  "  <title>%s</title>\n"
  "  <link rel=\"stylesheet\" type=\"text/css\" href=\"/static/webview.css\" />\n"
  " </head>\n"
  " <body>\n"
  "  <div id=\"header\">"
  "<a id=\"logo\" href=\"/\"/><img src=\"/static/webview.png\" alt=\"Fawkes WebView\"/></a>"
  "<hr /></div>\n";

/** Page footer template. */
const char *  WebPageReply::PAGE_FOOTER =
  "\n </body>\n"
  "</html>\n";

std::map<std::string, std::string> WebPageReply::__nav_entries;
std::string WebPageReply::__current_baseurl;


/** Constructor.
 * @param title title of the page
 * @param body Optional initial body of the page
 */
WebPageReply::WebPageReply(std::string title, std::string body)
  : StaticWebReply(WebReply::HTTP_OK, body)
{
  _title = title;
}


/** Base constructor.
 * Constructor that does not set a title or anything. Use for sub-classes.
 * @param code HTTP code for this reply
 */
WebPageReply::WebPageReply(response_code_t code)
  : StaticWebReply(code)
{
}


void
WebPageReply::pack()
{
  __merged_body  = html_header(_title) + _body + PAGE_FOOTER;
}

std::string::size_type
WebPageReply::body_length()
{
  return __merged_body.length();
}


const std::string &
WebPageReply::body()
{
  return __merged_body;
}


/** Generate HTML header.
 * Generate basic HTML header for the body and generate the navigation in
 * the page header.
 * @param title title of the page
 * @return generated header
 */
std::string
WebPageReply::html_header(std::string &title)
{
  std::string rv = "";
  char *s;
  if ( asprintf(&s, PAGE_HEADER, title.c_str()) != -1 ) {
    rv = s;
    free(s);
  }

  rv += "  <div id=\"mainnav\" class=\"nav\"><ul>";
  std::map<std::string, std::string>::iterator nei;
  for (nei = __nav_entries.begin(); nei != __nav_entries.end(); ++nei) {
    rv += "<li";
    if ( nei->first == __current_baseurl ) {
      rv += " class=\"active\"";
    }
    rv += "><a href=\"" + nei->first + "\">" + nei->second + "</a></li>";
  }
  rv += "</ul></div>";

  return rv;
}


/** Generate HTML footer.
 * Generate basic HTML footer and possibly additional footer content.
 * @return generated footer
 */
std::string
WebPageReply::html_footer()
{
  return PAGE_FOOTER;
}


/** Add navigation entry.
 * @param baseurl baseurl that should be linked for this entry
 * @param name string to print as link name
 */
void
WebPageReply::add_nav_entry(std::string baseurl, std::string name)
{
  __nav_entries[baseurl] = name;
}


/** Remove navigation entry.
 * @param baseurl baseurl whose config entry to remove
 */
void
WebPageReply::remove_nav_entry(std::string baseurl)
{
  __nav_entries.erase(baseurl);
}


/** Set current active baseurl.
 * @param baseurl new currently active baseurl
 */
void
WebPageReply::set_active_baseurl(std::string baseurl)
{
  __current_baseurl = baseurl;
}
