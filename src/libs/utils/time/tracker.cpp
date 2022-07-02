
/***************************************************************************
 *  tracker.cpp - Implementation of time tracker
 *
 *  Created: Fri Jun 03 13:43:33 2005 (copied from RCSoft5 FireVision)
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

#include <core/exceptions/software.h>
#include <core/exceptions/system.h>
#include <utils/time/tracker.h>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>

using namespace std;

namespace fawkes {

/** @class TimeTracker <utils/time/tracker.h>
 * Time tracking utility.
 * This class provides means to track time of different tasks in a process.
 * You can assign an arbitrary number of tracking classes per object (although
 * using a few classes is recommended for minimal influence of the measurement
 * on the measured process). You can then print out averages and (max) deviation
 * to get a feeling for the average performance and how flaky the runtimes are.
 *
 * The time tracker can also be operated without any class if you only want to
 * track a single process.
 *
 * You can either just ping classless or a specific class which will then take
 * the time difference between now and the last ping as the measured time. This
 * is useful to determine the call frequency of a given item.
 * If you want to benchmark sub-tasks you more likely want to start measuring at
 * a specific point in time and then stop it after the sub-task is done to measure
 * only this very task. This can be done by using pingStart() and pingEnd().
 *
 * @author Tim Niemueller
 */

/** The default tracking class. Optionally added in the constructor. */
const unsigned int TimeTracker::DEFAULT_CLASS = 0;

/** Constructor.
 * @param add_default_class if true a default time class is added.
 */
TimeTracker::TimeTracker(bool add_default_class)
{
	timelog_     = NULL;
	write_cycle_ = 0;
	reset();
	if (add_default_class) {
		class_times_.push_back(vector<struct timeval *>());
		class_names_.push_back("Default");
	}
}

/** Constructor for file logging.
 * @param filename name of the file to write log data to. File is overwritten.
 * @param add_default_class if true a default time class is added.
 */
TimeTracker::TimeTracker(const char *filename, bool add_default_class)
{
	write_cycle_ = 0;
	reset();
	if (add_default_class) {
		class_times_.push_back(vector<struct timeval *>());
		class_names_.push_back("Default");
	}
	timelog_ = fopen(filename, "w");
	if (!timelog_) {
		throw CouldNotOpenFileException(filename, errno, "Failed to open time log");
	}
}

/** Destructor. */
TimeTracker::~TimeTracker()
{
	if (timelog_) {
		fclose(timelog_);
	}
	reset();
	class_times_.clear();
	class_names_.clear();
}

/** Reset times.
 * Reset tracker and set comment.
 * @param comment comment to set on tracker.
 */
void
TimeTracker::reset(std::string comment)
{
	tracker_comment_ = comment;
	for (vector<vector<struct timeval *>>::iterator i = class_times_.begin(); i != class_times_.end();
	     ++i) {
		for (vector<struct timeval *>::iterator j = i->begin(); j != i->end(); ++j) {
			free(*j);
		}
		i->clear();
	}
	times_.clear();
	comments_.clear();
	gettimeofday(&start_time, NULL);
	gettimeofday(&last_time, NULL);
}

/** Ping classless.
 * This takes the time difference between now and the last ping and adds this
 * to classless tracking.
 * @param comment optional ping comment.
 */
void
TimeTracker::ping(std::string comment)
{
	timeval *t = (timeval *)malloc(sizeof(timeval));
	gettimeofday(t, NULL);
	times_.push_back(t);
	if (!comment.empty()) {
		comments_[times_.size() - 1] = comment;
	}
}

/** Add a new class.
 * Adds a new class and gives the class ID.
 * @param name name of the class
 * @return new class ID which is used for pinging this specific
 * class.
 */
unsigned int
TimeTracker::add_class(std::string name)
{
	if (name == "") {
		throw Exception("TimeTracker::add_class(): Class name may not be empty");
	}
	class_times_.push_back(vector<struct timeval *>());
	class_names_.push_back(name);
	return class_times_.size() - 1;
}

/** Remove a class.
 * This marks the class as unused. It is not longer possible to add times to this
 * class but they will not be printed anymore. The space associated with this
 * class is freed.
 * @param cls ID of the class to remove
 */
void
TimeTracker::remove_class(unsigned int cls)
{
	if (cls < class_names_.size()) {
		class_names_[cls] = "";
	} else {
		if (class_times_.size() == 0) {
			throw Exception("No classes have been added, cannot delete class %u", cls);
		} else {
			throw OutOfBoundsException("Invalid class given", cls, 0, class_times_.size() - 1);
		}
	}
}

/** Ping class.
 * This takes the time difference between now and the last ping and adds this
 * to class cls.
 * @param cls class ID to ping
 */
void
TimeTracker::ping(unsigned int cls)
{
	timeval *t = (timeval *)malloc(sizeof(timeval));
	gettimeofday(t, NULL);

	long sec  = t->tv_sec - last_time.tv_sec;
	long usec = t->tv_usec - last_time.tv_usec;
	if (usec < 0) {
		sec -= 1;
		usec += 1000000;
	}
	last_time.tv_sec  = t->tv_sec;
	last_time.tv_usec = t->tv_usec;

	t->tv_sec  = sec;
	t->tv_usec = usec;

	if (cls < class_times_.size()) {
		class_times_[cls].push_back(t);
	} else {
		if (class_times_.size() == 0) {
			throw Exception("No classes have been added, cannot track times");
		} else {
			throw OutOfBoundsException("Invalid class given", cls, 0, class_times_.size() - 1);
		}
	}
}

/** Start of given class task.
 * Signal the start of the given class.
 * @param cls class ID
 */
void
TimeTracker::ping_start(unsigned int cls)
{
	if (cls >= class_times_.size())
		return;

	timeval *t = (timeval *)malloc(sizeof(timeval));
	gettimeofday(t, NULL);

	if (cls < class_times_.size()) {
		class_times_[cls].push_back(t);
	} else {
		if (class_times_.size() == 0) {
			throw Exception("No classes have been added, cannot track times");
		} else {
			throw OutOfBoundsException("Invalid class given", cls, 0, class_times_.size() - 1);
		}
	}
}

/** End of given class task.
 * This takes the time difference between now and the last pingStart() for the
 * class cls.
 * @param cls class ID to signal end of task
 */
void
TimeTracker::ping_end(unsigned int cls)
{
	if (cls >= class_times_.size())
		return;

	timeval t2;
	gettimeofday(&t2, NULL);

	timeval *t1 = class_times_[cls].back();

	long sec  = t2.tv_sec - t1->tv_sec;
	long usec = t2.tv_usec - t1->tv_usec;

	if (usec < 0) {
		sec -= 1;
		usec += 1000000;
	}

	t1->tv_sec  = sec;
	t1->tv_usec = usec;
}

/** End of given class task without recording.
 * End the duration but do not take the time into the result measurements.
 * @param cls class ID to signal end of task
 */
void
TimeTracker::ping_abort(unsigned int cls)
{
	if (cls >= class_times_.size())
		return;

	free(class_times_[cls].back());
	class_times_[cls].pop_back();
}

void
TimeTracker::average_and_deviation(vector<struct timeval *> &values,
                                   double                   &average_sec,
                                   double                   &average_ms,
                                   double                   &deviation_sec,
                                   double                   &deviation_ms)
{
	vector<struct timeval *>::iterator tit;

	average_sec = average_ms = deviation_sec = deviation_ms = 0.f;

	for (tit = values.begin(); tit != values.end(); ++tit) {
		average_sec += float((*tit)->tv_sec);
		average_sec += (*tit)->tv_usec / 1000000.f;
	}
	average_sec /= values.size();

	for (tit = values.begin(); tit != values.end(); ++tit) {
		deviation_sec += fabs((*tit)->tv_sec + ((*tit)->tv_usec / 1000000.f) - average_sec);
	}
	deviation_sec /= values.size();

	average_ms   = average_sec * 1000.f;
	deviation_ms = deviation_sec * 1000.f;
}

/** Print results to stdout. */
void
TimeTracker::print_to_stdout()
{
	if (!times_.empty()) {
		unsigned int i               = 0;
		unsigned int j               = 0;
		long         diff_sec_start  = 0;
		long         diff_usec_start = 0;
		long         diff_sec_last   = 0;
		long         diff_usec_last  = 0;
		float        diff_msec_start = 0.0;
		float        diff_msec_last  = 0.0;
		time_t       last_sec        = start_time.tv_sec;
		suseconds_t  last_usec       = start_time.tv_usec;
		char         time_string[26];

		ctime_r(&(start_time.tv_sec), time_string);
		for (j = 26; j > 0; --j) {
			if (time_string[j] == '\n') {
				time_string[j] = 0;
				break;
			}
		}

		cout << endl << "TimeTracker stats - individual times";
		if (tracker_comment_.empty()) {
			cout << " (" << tracker_comment_ << ")";
		}
		cout << endl
		     << "==================================================================" << endl
		     << "Initialized: " << time_string << " (" << start_time.tv_sec << ")" << endl
		     << endl;

		for (time_it_ = times_.begin(); time_it_ != times_.end(); ++time_it_) {
			char tmp[10];
			sprintf(tmp, "%3u.", i + 1);
			cout << tmp;
			if (comments_.count(i) > 0) {
				cout << "  (" << comments_[i] << ")";
			}
			cout << endl;

			diff_sec_start  = (*time_it_)->tv_sec - start_time.tv_sec;
			diff_usec_start = (*time_it_)->tv_usec - start_time.tv_usec;
			if (diff_usec_start < 0) {
				diff_sec_start -= 1;
				diff_usec_start = 1000000 + diff_usec_start;
			}
			diff_msec_start = diff_usec_start / 1000.f;

			diff_sec_last  = (*time_it_)->tv_sec - last_sec;
			diff_usec_last = (*time_it_)->tv_usec - last_usec;
			if (diff_usec_last < 0) {
				diff_sec_last -= 1;
				diff_usec_last = 1000000 + diff_usec_last;
			}
			diff_msec_last = diff_usec_last / 1000.f;

			last_sec  = (*time_it_)->tv_sec;
			last_usec = (*time_it_)->tv_usec;

			ctime_r(&(*time_it_)->tv_sec, time_string);
			for (j = 26; j > 0; --j) {
				if (time_string[j] == '\n') {
					time_string[j] = 0;
					break;
				}
			}
			cout << time_string << " (" << (*time_it_)->tv_sec << ")" << endl;
			cout << "Diff to start: " << diff_sec_start << " sec and " << diff_usec_start
			     << " usec  (which are " << diff_msec_start << " msec)" << endl;
			cout << "Diff to last:  " << diff_sec_last << " sec and " << diff_usec_last
			     << " usec (which are " << diff_msec_last << " msec)" << endl
			     << endl;

			i += 1;
		}
	}

	cout << endl << "TimeTracker stats - class times";
	if (!tracker_comment_.empty()) {
		cout << " (" << tracker_comment_ << ")";
	}
	cout << endl << "==================================================================" << endl;

	vector<vector<struct timeval *>>::iterator it  = class_times_.begin();
	vector<string>::iterator                   sit = class_names_.begin();

	double deviation    = 0.f;
	double average      = 0.f;
	double average_ms   = 0.f;
	double deviation_ms = 0.f;

	for (; (it != class_times_.end()) && (sit != class_names_.end()); ++it, ++sit) {
		if (sit->empty())
			continue;

		if (it->size() > 0) {
			average_and_deviation(*it, average, average_ms, deviation, deviation_ms);

			cout << "Class '" << *sit << "'" << endl
			     << "  avg=" << average << " (" << average_ms << " ms)" << endl
			     << "  dev=" << deviation << " (" << deviation_ms << " ms)" << endl
			     << "  res=" << it->size() << " results" << endl;
		} else {
			cout << "Class '" << *sit << "' has no results." << endl;
		}
	}

	cout << endl;
}

/** Print data to file suitable for gnuplot.
 * This will write the following data:
 * average sec, average ms, average summed sec, deviation sec, deviation ms
 * This data is generated for each class and concatenated into a single line
 * and written to the file. A running number will be prepended as the first
 * value. The data file is suitable as input for gnuplot.
 */
void
TimeTracker::print_to_file()
{
	if (!timelog_)
		throw Exception("Time log not opened, use other ctor");

	vector<vector<struct timeval *>>::iterator it  = class_times_.begin();
	vector<string>::iterator                   sit = class_names_.begin();

	double deviation    = 0.f;
	double average      = 0.f;
	double average_ms   = 0.f;
	double deviation_ms = 0.f;
	double avgsum       = 0.f;

	fprintf(timelog_, "%u ", ++write_cycle_);
	for (; (it != class_times_.end()) && (sit != class_names_.end()); ++it, ++sit) {
		if (sit->empty())
			continue;

		average_and_deviation(*it, average, average_ms, deviation, deviation_ms);

		avgsum += average;
		fprintf(timelog_, "%lf %lf %lf %lf %lf ", average, average_ms, avgsum, deviation, deviation_ms);
	}
	fprintf(timelog_, "\n");
	fflush(timelog_);
}

/** @class ScopedClassItemTracker "utils/time/tracker.h"
 * Scoped time tracking for specific item.
 * @author Victor Matare
 */

/** Constructor.
 * Starts time tracking for given class on given time tracker.
 * @param tt time tracker
 * @param cls class ID
 */
ScopedClassItemTracker::ScopedClassItemTracker(TimeTracker &tt, unsigned int cls)
: tt_(tt), cls_(cls)
{
	tt_.ping_start(cls_);
}

/** Destructor. */
ScopedClassItemTracker::~ScopedClassItemTracker()
{
	tt_.ping_end(cls_);
}

} // end namespace fawkes
