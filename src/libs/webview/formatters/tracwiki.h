
/***************************************************************************
 *  tracwiki.h - Trac wiki style formatter
 *
 *  Created: Wed May 11 17:02:18 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __LIBS_WEBVIEW_FORMATTERS_TRACWIKI_H_
#define __LIBS_WEBVIEW_FORMATTERS_TRACWIKI_H_

#include <regex.h>
#include <string>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class TracWikiHeadingFormatter
{
 public:
  TracWikiHeadingFormatter();
  virtual ~TracWikiHeadingFormatter();

  virtual std::string format(std::string &text);

 private:
  regex_t re_heading_;
};

} // end namespace fawkes

#endif
