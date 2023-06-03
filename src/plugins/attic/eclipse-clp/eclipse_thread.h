
/***************************************************************************
 *  eclipse_thread.h - Fawkes ECLiPSe Thread
 *
 *  Created: Wed Jul 16 10:20:51 2009
 *  Copyright  2009  Daniel Beck
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

#ifndef _PLUGINS_ECLIPSE_CLP_ECLIPSE_THREAD_H_
#define _PLUGINS_ECLIPSE_CLP_ECLIPSE_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/mutex.h>
#include <core/threading/thread.h>

#include <eclipseclass.h>
#include <string>
#include <vector>

namespace fawkes {
class Interface;
class Mutex;
} // namespace fawkes

class EclipseAgentThread : public fawkes::Thread,
                           public fawkes::BlackBoardAspect,
                           public fawkes::ConfigurableAspect,
                           public fawkes::LoggingAspect
{
public:
	EclipseAgentThread();
	virtual ~EclipseAgentThread();

	virtual void init();
	virtual void finalize();
	virtual void once();
	virtual void loop();

	void            post_event(const char *);
	fawkes::Logger *get_logger();

	static EclipseAgentThread *instance();

private: /* methods */
	bool load_file(const char *filename);

private: /* members */
	static EclipseAgentThread *m_instance;

	bool           m_initialized;
	std::string    agent;
	std::string    graph_path;
	int            ec_result;
	EC_ref         ec_yield_reason;
	fawkes::Mutex *mutex;
};

#endif /* PLUGINS_ECLIPSE_CLP_ECLIPSE_THREAD_H__ */
