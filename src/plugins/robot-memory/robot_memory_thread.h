
/***************************************************************************
 *  robot_memory_thread.h - Robot Memory thread
 *
 *  Created: Sun May 01 13:39:52 2016
 *  Copyright  2016  Frederik Zwilling
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

#ifndef __PLUGINS_ROBOT_MEMORY_THREAD_H_
#define __PLUGINS_ROBOT_MEMORY_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <plugins/mongodb/aspect/mongodb.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <mongo/client/dbclient.h>

#include <string>

namespace fawkes {
  class Mutex;
  class RobotMemoryInterface;
}

class RobotMemoryThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::MongoDBAspect,
	public fawkes::BlockedTimingAspect,
  public fawkes::BlackBoardAspect
{
 public:
  RobotMemoryThread();
  virtual ~RobotMemoryThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  std::string    __collection;
  fawkes::Mutex *__mutex;
  fawkes::RobotMemoryInterface *__rm_if;

  void exec_query(std::string query, std::string collection);
  void exec_query(std::string query);
  void exec_insert(std::string insert, std::string collection);
  void exec_insert(std::string insert);
  void exec_update(std::string query, std::string update, std::string collection);
  void exec_update(std::string query, std::string update);
  void exec_remove(std::string query, std::string collection);
  void exec_remove(std::string query);

  void log(mongo::Query query, std::string what);
  void log(mongo::BSONObj obj, std::string what);

  void set_fields(mongo::BSONObj &obj, std::string what);
  void set_fields(mongo::Query &q, std::string what);
  void remove_field(mongo::Query &q, std::string what);

  //funtions to generate virtual knowledge
  void gen_blackboard_data(std::string field);
};

#endif
