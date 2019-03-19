
/***************************************************************************
 *  log_stream.h - Route Plexil log output to Fawkes log
 *
 *  Created: Tue Aug 14 15:04:57 2018
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
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

#include <logging/logger.h>

#include <iostream>
#include <sstream>

/** Log Plexil log output to Fawkes logger.
 * @author Tim Niemueller
 */
class PlexilLogStreamBuffer : public std::basic_streambuf<char>
{
public:
	/** Constructor.
	 * @param logger logger to write output to
	 */
	PlexilLogStreamBuffer(fawkes::Logger *logger) : logger_(logger)
	{
	}

	/** Consume characters.
	 * @param ch character to add
	 * @return always zero
	 */
	int_type
	overflow(int_type ch)
	{
		if (ch == '\n') {
			std::string            s = ss_.str();
			std::string::size_type opening, closing;
			opening = s.find('[');
			closing = s.find(']');
			if (opening != std::string::npos && closing != std::string::npos) {
				std::string comp = "Plexil" + s.substr(opening, closing - opening + 1);
				std::string text = s.substr(closing + 1);
				logger_->log_info(comp.c_str(), "%s", text.c_str());
			} else {
				logger_->log_info("Plexil", "%s", s.c_str());
			}
			ss_.str("");
		} else {
			ss_.put(ch);
		}
		return 0;
	}

private:
	fawkes::Logger *  logger_;
	std::stringstream ss_;
};
