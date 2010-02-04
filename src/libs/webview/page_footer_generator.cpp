
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

#include <webview/page_footer_generator.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class WebPageFooterGenerator <webview/page_footer_generator.h>
 * Interface for HTML footer generator.
 * A page footer generator has the task to generate the HTML code that is
 * appended to each standard page. This is a possible footer or navigational
 * additions. A footer generator must also generate the closing
 * &lt;/body&gt;&lt;/html&gt; tags.
 * @author Tim Niemueller
 *
 * @fn std::string WebPageFooterGenerator::html_footer() = 0
 * Generate HTML footer.
 * @return footer HTML code, including &lt;/body&gt;&lt;/html&gt;
 */

/** Virtual empty destructor. */
WebPageFooterGenerator::~WebPageFooterGenerator()
{
}

} // end namespace fawkes
