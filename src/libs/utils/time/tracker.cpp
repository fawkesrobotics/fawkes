
/***************************************************************************
 *  tracker.cpp - Implementation of time tracker
 *
 *  Created: Fri Jun 03 13:43:33 2005 (copied from RCSoft5 FireVision)
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <utils/time/tracker.h>
#include <utils/system/console_colors.h>

#include <iostream>
#include <cmath>

using namespace std;

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


/** Constructor.
 * Creates a default time class.
 */
TimeTracker::TimeTracker()
{
  reset();
  classTimes.push_back( new vector< struct timeval * > );
  classNames.push_back( new string("Default") );
}


/** Destructor. */
TimeTracker::~TimeTracker()
{
  reset();
  vector< vector< struct timeval * > * >::iterator it;
  vector< struct timeval * >::iterator tit;
  for (it = classTimes.begin(); it != classTimes.end(); ++it) {
    for (tit = (*it)->begin(); tit != (*it)->end(); ++tit) {
      free(*tit);
    }
    (*it)->clear();
    delete (*it);
  }
  vector< string * >::iterator sit;
  for (sit = classNames.begin(); sit != classNames.end(); ++sit) {
    delete (*sit);
  }
  classTimes.clear();
  classNames.clear();
}


/** Reset times.
 * Reset tracker and set comment.
 * @param comment comment to set on tracker.
 */
void
TimeTracker::reset(string comment)
{
  tracker_comment = comment;
  for (time_it = times.begin(); time_it != times.end(); ++time_it) {
    free(*time_it);
  }
  for (comment_it = comments.begin(); comment_it != comments.end(); ++comment_it) {
    delete comment_it->second;
  }
  times.clear();
  comments.clear();
  gettimeofday(&start_time, NULL);
  gettimeofday(&last_time, NULL);
}


/** Ping classless.
 * This takes the time difference between now and the last ping and adds this
 * to classless tracking.
 * @param comment optional ping comment.
 */
void
TimeTracker::ping(string comment)
{
  timeval *t = (timeval *)malloc(sizeof(timeval));
  gettimeofday(t, NULL);
  times.push_back(t);
  if (comment.length() > 0) {
    comments[ times.size() - 1 ] = new string(comment);
  }
}


/** Add a new class.
 * Adds a new class and gives the class ID.
 * @param name name of the class
 * @return new class ID which is used for pinging this specific
 * class.
 */
unsigned int
TimeTracker::add_class(string name)
{
  classTimes.push_back( new vector< struct timeval * > );
  classNames.push_back( new string(name) );
  return classTimes.size() - 1;
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

  if (cls < classTimes.size()) {
    classTimes[cls]->push_back(t);
  } else {
    classTimes[0]->push_back(t);
  }
}


/** Start of given class task.
 * Signal the start of the given class.
 * @param cls class ID
 */
void
TimeTracker::ping_start(unsigned int cls)
{
  if (cls >= classTimes.size()) return;

  timeval *t = (timeval *)malloc(sizeof(timeval));
  gettimeofday(t, NULL);

  if (cls < classTimes.size()) {
    classTimes[cls]->push_back(t);
  } else {
    classTimes[0]->push_back(t);
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
  if (cls >= classTimes.size()) return;

  timeval t2;
  gettimeofday(&t2, NULL);

  timeval *t1 = classTimes[cls]->back();

  long sec  = t2.tv_sec - t1->tv_sec;
  long usec = t2.tv_usec - t1->tv_usec;

  if (usec < 0) {
    sec  -= 1;
    usec += 1000000;
  }

  t1->tv_sec  = sec;
  t1->tv_usec = usec;

}


/** Print results to stdout. */
void
TimeTracker::print_to_stdout()
{

  unsigned int i = 0;
  unsigned int j = 0;
  long diff_sec_start = 0;
  long diff_usec_start = 0;
  long diff_sec_last = 0;
  long diff_usec_last = 0;
  float diff_msec_start = 0.0;
  float diff_msec_last = 0.0;
  time_t last_sec = start_time.tv_sec;
  suseconds_t last_usec = start_time.tv_usec;
  char time_string[26];

  ctime_r(&(start_time.tv_sec), time_string);
  for (j = 26; j > 0; --j) {
    if (time_string[j] == '\n') {
      time_string[j] = 0;
      break;
    }
  }

  cout << endl << "TimeTracker stats - individual times";
  if (tracker_comment.length() > 0) {
    cout << " (" << tracker_comment << ")";
  }
  cout << endl
       << "==================================================================" << endl
       << "Initialized: " << time_string << " (" << start_time.tv_sec << ")" << endl << endl;

  for (time_it = times.begin(); time_it != times.end(); ++time_it) {
    char tmp[10];
    sprintf(tmp, "%3u.", i + 1);
    cout << tmp;
    if (comments.count(i) > 0) {
      cout << "  (" << *comments[i] << ")";
    }
    cout << endl;

    diff_sec_start  = (*time_it)->tv_sec  - start_time.tv_sec;
    diff_usec_start = (*time_it)->tv_usec - start_time.tv_usec;
    if (diff_usec_start < 0) {
      diff_sec_start -= 1;
      diff_usec_start = 1000000 + diff_usec_start;
    }
    diff_msec_start = diff_usec_start / 1000.f;

    diff_sec_last  = (*time_it)->tv_sec  - last_sec;
    diff_usec_last = (*time_it)->tv_usec - last_usec;
    if (diff_usec_last < 0) {
      diff_sec_last -= 1;
      diff_usec_last = 1000000 + diff_usec_last;
    }
    diff_msec_last = diff_usec_last / 1000.f;

    last_sec  = (*time_it)->tv_sec;
    last_usec = (*time_it)->tv_usec;

    ctime_r(&(*time_it)->tv_sec, time_string);
    for (j = 26; j > 0; --j) {
      if (time_string[j] == '\n') {
	time_string[j] = 0;
	break;
      }
    }
    cout << time_string << " (" << (*time_it)->tv_sec << ")" << endl;
    cout << "Diff to start: " << diff_sec_start << " sec and " << diff_usec_start
	 << " usec  (which are "
	 << diff_msec_start << " msec)" << endl;
    cout << "Diff to last:  " << diff_sec_last  << " sec and " << diff_usec_last
	 << " usec (which are "
	 << diff_msec_last << " msec)" << endl << endl;

    i += 1;
  }

  cout << endl << "TimeTracker stats - class times";
  if (tracker_comment.length() > 0) {
    cout << " (" << tracker_comment << ")";
  }
  cout << endl
       << "==================================================================" << endl;

  vector< vector< struct timeval * > * >::iterator it = classTimes.begin();
  vector< struct timeval * >::iterator tit;
  vector< string * >::iterator sit = classNames.begin();

  double deviation = 0.f;
  double average = 0.f;
  double average_ms = 0.f;
  double deviation_ms = 0.f;

  for (; (it != classTimes.end()) && (sit != classNames.end()); ++it, ++sit) {
    deviation = 0.f;
    average = 0.f;

    if ((*it)->size() > 0) {
      for (tit = (*it)->begin(); tit != (*it)->end(); ++tit) {
	average += float((*tit)->tv_sec);
	average += (*tit)->tv_usec / 1000000.f;
      }
      average /= (*it)->size();

      for (tit = (*it)->begin(); tit != (*it)->end(); ++tit) {
	deviation += fabs((*tit)->tv_sec + ((*tit)->tv_usec / 1000000.f) - average);
      }
      deviation /= (*it)->size();
    }

    average_ms = average * 1000;
    deviation_ms = deviation * 1000;

    cout << "Class '" <<  **sit << "'" << endl
	 << "  avg=" << average << " (" << average_ms << " ms)" << endl
	 << "  dev=" << deviation << " (" << deviation_ms << " ms)" << endl
	 << "  res=" << (*it)->size() << " results"
	 << endl;

  }

  cout << endl;

}
