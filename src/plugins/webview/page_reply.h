
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

#ifndef __PLUGINS_WEBVIEW_PAGE_REPLY_H_
#define __PLUGINS_WEBVIEW_PAGE_REPLY_H_

#include "reply.h"

class WebPageReply : public StaticWebReply
{
  friend class WebRequestDispatcher;
  friend class WebviewThread;
 public:
  WebPageReply(std::string title, std::string page = "");

  static std::string html_header(std::string &title);
  static std::string html_footer();

  virtual const std::string & body();
  virtual std::string::size_type body_length();
  virtual void pack();

 protected:
  WebPageReply(response_code_t code);

 protected:
  /** Title of the page. */
  std::string _title;

 private:
  static void add_nav_entry(std::string baseurl, std::string name);
  static void remove_nav_entry(std::string baseurl);
  static void set_active_baseurl(std::string baseurl);

 private:
  static const char *PAGE_HEADER;
  static const char *PAGE_FOOTER;

  std::string __merged_body;

  static std::map<std::string, std::string> __nav_entries;
  static std::string __current_baseurl;
};

#endif
