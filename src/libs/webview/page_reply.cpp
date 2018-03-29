
/***************************************************************************
 *  page_reply.h - Web request reply for a normal page
 *
 *  Created: Thu Oct 23 16:13:48 2008
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

#include <webview/page_reply.h>
#include <webview/page_header_generator.h>
#include <webview/page_footer_generator.h>
#include <utils/system/hostinfo.h>

#include <cstdlib>
#include <cstring>
#include <cstdio>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class WebPageReply <webview/page_reply.h>
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
  "  <link rel=\"stylesheet\" type=\"text/css\" href=\"/static/css/webview.css\" />\n"
  "%s"
  " </head>\n"
  " <body>\n";

/** Page footer template. */
const char *  WebPageReply::PAGE_FOOTER =
  "\n </body>\n"
  "</html>\n";

/** Constructor.
 * @param title title of the page
 * @param body Optional initial body of the page
 */
WebPageReply::WebPageReply(std::string title, std::string body)
  : StaticWebReply(WebReply::HTTP_OK, body)
{
  _title = title;
  navbar_enabled_ = true;
  footer_enabled_ = true;

  add_header("Content-type", "text/html");
}


/** Base constructor.
 * Constructor that does not set a title or anything. Use for sub-classes.
 * @param code HTTP code for this reply
 */
WebPageReply::WebPageReply(Code code)
  : StaticWebReply(code)
{
  navbar_enabled_ = true;
  footer_enabled_ = true;

  add_header("Content-type", "text/html");
}


/** Set HTML header text.
 * The given text is placed in the head section of the HTML page. You can use it
 * for example to add custom stylesheets or JavaScript.
 * @param h header to set
 */
void
WebPageReply::set_html_header(std::string h)
{
  html_header_ = h;
}


/** Pack web page reply.
 * This method creates the final page by calling the header and footer generators
 * if supplied (otherwise a standard header is chosen) and the body.
 * @param active_baseurl the active navigation URL, can be used for instance
 * to high-light the current section in the navigation.
 * @param headergen header generator
 * @param footergen footer generator
 */
void
WebPageReply::pack(std::string active_baseurl,
		   WebPageHeaderGenerator *headergen,
		   WebPageFooterGenerator *footergen)
{
  if (headergen && navbar_enabled_)
    merged_body_ += headergen->html_header(_title, active_baseurl, html_header_);
  else {
    fawkes::HostInfo hi;
    char *s;
    if ( asprintf(&s, PAGE_HEADER, _title.c_str(), html_header_.c_str(), hi.short_name()) != -1 ) {
      merged_body_ += s;
      free(s);
    }
  }

  merged_body_  += _body;

  if (footergen && footer_enabled_)  merged_body_ += footergen->html_footer();
  else            merged_body_ += PAGE_FOOTER;
}

std::string::size_type
WebPageReply::body_length()
{
  return merged_body_.length();
}


const std::string &
WebPageReply::body()
{
  return merged_body_;
}

/** 
 * Enable or disable the Fawkes Navigationbar (default enabled)
 * @param enabled enabled
 */
void WebPageReply::set_navbar_enabled(bool enabled)
{
  navbar_enabled_ = enabled;
}
/**
 * Is the Fawkes Navigation bar enabled?
 * @return enabled
 */
bool WebPageReply::get_navbar_enabled()
{
  return navbar_enabled_;
}
/** 
 * Enable or disable the Fawkes Webview footer (default enabled)
 * @param enabled enabled
 */
void WebPageReply::set_footer_enabled(bool enabled)
{
  footer_enabled_ = enabled;
}
/**
 * Is the Fawkes Webview footer enabled?
 * @return enabled
 */
bool WebPageReply::get_footer_enabled()
{
  return footer_enabled_;
}


} // end namespace fawkes
