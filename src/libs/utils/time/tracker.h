
/***************************************************************************
 *  tracker.h - Header for time tracker, which can be used to track a
 *              process's times
 *
 *  Created: Fri Jun 03 13:43:20 2005 (copied from RCSoft5 FireVision)
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

#ifndef __UTILS_TIME_TRACKER_H_
#define __UTILS_TIME_TRACKER_H_

#include <vector>
#include <map>
#include <string>
#include <sys/time.h>

class TimeTracker {
 public:
  TimeTracker();
  ~TimeTracker();

  unsigned int addClass(std::string name);

  void ping(unsigned int cls);
  void pingStart(unsigned int cls);
  void pingEnd(unsigned int cls);

  void ping(std::string comment = "");
  void reset(std::string comment = "");
  void printToStdout();
  
 private:
  timeval start_time;
  timeval last_time;
  std::vector< std::vector< struct timeval * > * > classTimes;
  std::vector< std::string * > classNames;
  std::vector< struct timeval * > times;
  std::map< unsigned int, std::string * > comments;
  std::vector< struct timeval * >::iterator time_it;
  std::map< unsigned int, std::string * >::iterator comment_it;
  std::string tracker_comment;
};




#endif
