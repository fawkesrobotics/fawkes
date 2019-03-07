
/***************************************************************************
 *  tracker.h - Time tracker, which can be used to track a process's times
 *
 *  Created: Fri Jun 03 13:43:20 2005 (copied from RCSoft5 FireVision)
 *  Copyright  2005-2009  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef _UTILS_TIME_TRACKER_H_
#define _UTILS_TIME_TRACKER_H_

#include <sys/time.h>

#include <cstdio>
#include <map>
#include <string>
#include <vector>

namespace fawkes {

class TimeTracker
{
public:
	static const unsigned int DEFAULT_CLASS;

	TimeTracker(const char *filename, bool add_default_class = false);
	TimeTracker(bool add_default_class = false);
	~TimeTracker();

	unsigned int add_class(std::string name);
	void         remove_class(unsigned int cls);

	void ping(unsigned int cls);
	void ping_start(unsigned int cls);
	void ping_end(unsigned int cls);
	void ping_abort(unsigned int cls);

	void ping(std::string comment = "");
	void reset(std::string comment = "");
	void print_to_stdout();

	void print_to_file();

private:
	void average_and_deviation(std::vector<struct timeval *> &values,
	                           double &                       average_sec,
	                           double &                       average_ms,
	                           double &                       deviation_sec,
	                           double &                       deviation_ms);

private:
	timeval                                       start_time;
	timeval                                       last_time;
	std::vector<std::vector<struct timeval *>>    class_times_;
	std::vector<std::string>                      class_names_;
	std::vector<struct timeval *>                 times_;
	std::map<unsigned int, std::string>           comments_;
	std::vector<struct timeval *>::iterator       time_it_;
	std::map<unsigned int, std::string>::iterator comment_it_;
	std::string                                   tracker_comment_;

	unsigned int write_cycle_;
	FILE *       timelog_;
};

class ScopedClassItemTracker
{
public:
	explicit ScopedClassItemTracker(TimeTracker &tt, unsigned int cls);
	~ScopedClassItemTracker();

private:
	TimeTracker &tt_;
	unsigned int cls_;
};

} // end namespace fawkes

#endif
