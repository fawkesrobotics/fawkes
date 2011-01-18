
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

#include <webview/page_header_generator.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class WebPageHeaderGenerator <webview/page_header_generator.h>
 * Interface for HTML header generator.
 * A page header generator has the task to generate the HTML code that is
 * prepended to each standard page. This is a possible header or navigational
 * additions. A header generator must also generate the opening "&lt;html&gt;"
 * tags, the &lt;head&gt; section, and the opening &lt;body&gt; tag.
 * @author Tim Niemueller
 *
 * @fn std::string WebPageHeaderGenerator::html_header(std::string &title, std::string &active_baseurl, std::string &html_header) = 0
 * Generate HTML header.
 * @param title HTML title, must be put in &lt;title&gt; tag in &lt;head&gt;
 * section
 * @param active_baseurl the baseurl currently active processor (this is the
 * baseurl a processor is registered for). This can be used for example to
 * highlight the current section in the navigation.
 * @param html_header custom HTML code to place in the head element of the page.
 * @return header HTML code
 */

WebPageHeaderGenerator::~WebPageHeaderGenerator()
{
}

} // end namespace fawkes
