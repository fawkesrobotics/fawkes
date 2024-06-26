
/***************************************************************************
 *  fam.h - File Alteration Monitor
 *
 *  Created: Fri May 23 11:38:41 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef _UTILS_SYSTEM_FAM_H_
#define _UTILS_SYSTEM_FAM_H_

#include <core/utils/lock_list.h>
#include <sys/types.h>

#include <map>
#include <regex.h>
#include <string>

namespace fawkes {

class FamListener
{
public:
	virtual ~FamListener();

	static const unsigned int FAM_ACCESS;
	static const unsigned int FAM_MODIFY;
	static const unsigned int FAM_ATTRIB;
	static const unsigned int FAM_CLOSE_WRITE;
	static const unsigned int FAM_CLOSE_NOWRITE;
	static const unsigned int FAM_CLOSE;
	static const unsigned int FAM_OPEN;
	static const unsigned int FAM_MOVED_FROM;
	static const unsigned int FAM_MOVED_TO;
	static const unsigned int FAM_MOVE;
	static const unsigned int FAM_CREATE;
	static const unsigned int FAM_DELETE;
	static const unsigned int FAM_DELETE_SELF;
	static const unsigned int FAM_MOVE_SELF;

	static const unsigned int FAM_UNMOUNT;
	static const unsigned int FAM_Q_OVERFLOW;
	static const unsigned int FAM_IGNORED;

	static const unsigned int FAM_ONLYDIR;
	static const unsigned int FAM_DONT_FOLLOW;
	static const unsigned int FAM_MASK_ADD;
	static const unsigned int FAM_ISDIR;
	static const unsigned int FAM_ONESHOT;

	static const unsigned int FAM_ALL_EVENTS;

	virtual void fam_event(const char *filename, unsigned int mask) = 0;
};

class FileAlterationMonitor
{
public:
	FileAlterationMonitor();
	~FileAlterationMonitor();

	void watch_dir(const char *dirpath);
	void watch_file(const char *filepath);
	void add_filter(const char *regex);
	void reset();

	void process_events(int timeout = 0);
	void interrupt();

	void add_listener(FamListener *listener);
	void remove_listener(FamListener *listener);

private:
	LockList<FamListener *>           listeners_;
	LockList<FamListener *>::iterator lit_;
	LockList<regex_t *>               regexes_;
	LockList<regex_t *>::iterator     rxit_;

	int    inotify_fd_;
	char  *inotify_buf_;
	size_t inotify_bufsize_;

	std::map<int, std::string>           inotify_watches_;
	std::map<int, std::string>::iterator inotify_wit_;

	bool interrupted_;
	bool interruptible_;
	int  pipe_fds_[2];
};

} // end of namespace fawkes

#endif
