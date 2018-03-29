
/***************************************************************************
 *  tracwiki.cpp - Trac wiki style formatter
 *
 *  Created: Wed May 11 17:04:04 2011
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

#include <webview/formatters/tracwiki.h>

#include <core/exception.h>
#include <sstream>
#include <cstring>
#include <cstdio>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class TracWikiHeadingFormatter <webview/formatters/tracwiki.h>
 * Translate Trac wiki heading syntax to HTML.
 * This class translates Trac wiki style heading identifications and
 * translates them into HTML.
 * @author Tim Niemueller
 */


/** Constructor. */
TracWikiHeadingFormatter::TracWikiHeadingFormatter()
{
  if ( regcomp(&re_heading_, "^(=+) (.*) \\1$", REG_EXTENDED) != 0 ) {
    throw Exception("Failed to compile heading regex");
  }

}


/** Destructor. */
TracWikiHeadingFormatter::~TracWikiHeadingFormatter()
{
  regfree(&re_heading_);
}


/** Format string.
 * @param text input text in Trac wiki markup language
 * @return formatted text
 */
std::string
TracWikiHeadingFormatter::format(std::string &text)
{
  std::string rv = "";

  bool in_paragraph = false;

  std::istringstream iss(text);
  while (!iss.eof() && !iss.fail()) {
    std::string line;
    getline(iss, line);
    const char *sl = line.c_str();

    int num_matches = 3;
    regmatch_t matches[num_matches];

    if (line == "") {
      if (in_paragraph) {
	rv += "</p>\n";
	in_paragraph = false;
      }
    } else if (regexec(&re_heading_, sl, num_matches, matches, 0) == 0) {
      unsigned int h_depth = matches[1].rm_eo - matches[1].rm_so;

      if (in_paragraph) {
	rv += "</p>\n";
	in_paragraph = false;
      }

      char title[matches[2].rm_eo - matches[2].rm_so + 1];
      title[matches[2].rm_eo - matches[2].rm_so] = 0;
      strncpy(title, &line.c_str()[matches[2].rm_so],
	      matches[2].rm_eo - matches[2].rm_so);
      char *tmp;
      if (asprintf(&tmp, "<h%u>%s</h%u>\n", h_depth, title, h_depth) != -1) {
	rv += tmp;
      } else {
	rv += line;
      }
    } else {
      if (!in_paragraph) {
	rv += "<p>";
	in_paragraph = true;
      }
      rv += line;
    }
  }

  if (in_paragraph) {
    rv += "</p>\n";
    in_paragraph = false;
  }

  return rv;
}

} // end namespace fawkes
