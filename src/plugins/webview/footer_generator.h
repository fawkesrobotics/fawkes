
/***************************************************************************
 *  footer_generator.h - Generator of page footer
 *
 *  Created: Sun Aug 30 14:37:21 2009
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

#ifndef __PLUGINS_WEBVIEW_FOOTER_GENERATOR_H_
#define __PLUGINS_WEBVIEW_FOOTER_GENERATOR_H_

#include <webview/page_footer_generator.h>

#include "service_browse_handler.h"

class WebviewFooterGenerator : public fawkes::WebPageFooterGenerator
{
 public:
  WebviewFooterGenerator(WebviewServiceBrowseHandler *service_browser);

  std::string html_footer();

 private:
  WebviewServiceBrowseHandler *service_browser_;
};

#endif
