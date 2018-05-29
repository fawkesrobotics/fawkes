
/***************************************************************************
 *  page_reply.h - Web request reply for a normal page
 *
 *  Created: Thu Oct 23 16:13:48 2008
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

#ifndef __LIBS_WEBVIEW_PAGE_REPLY_H_
#define __LIBS_WEBVIEW_PAGE_REPLY_H_

#include <webview/reply.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class WebPageHeaderGenerator;
class WebPageFooterGenerator;

class WebPageReply : public StaticWebReply
{
 public:
  WebPageReply(std::string title, std::string page = "");

  virtual const std::string & body();
  virtual std::string::size_type body_length();
  virtual void pack() { pack("", 0, 0); }
  virtual void pack(std::string active_baseurl,
		    WebPageHeaderGenerator *headergen,
		    WebPageFooterGenerator *footergen);

  virtual void set_html_header(std::string h);
  
  void set_navbar_enabled(bool enabled);
  bool get_navbar_enabled();
  void set_footer_enabled(bool enabled);
  bool get_footer_enabled();

 protected:
  WebPageReply(Code code);

 protected:
  /** Title of the page. */
  std::string _title;

 private:
  static const char *PAGE_HEADER;
  static const char *PAGE_FOOTER;

  std::string merged_body_;
  std::string html_header_;
  bool navbar_enabled_;
  bool footer_enabled_;
};

} // end namespace fawkes

#endif
