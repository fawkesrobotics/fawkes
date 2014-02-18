
/***************************************************************************
 *  mongorrd_thread.h - MongoDB RRD thread
 *
 *  Created: Sat Jan 15 18:42:12 2011
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

#ifndef __PLUGINS_MONGORRD_MONGORRD_THREAD_H_
#define __PLUGINS_MONGORRD_MONGORRD_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <plugins/mongodb/aspect/mongodb.h>
#include <plugins/rrd/aspect/rrd.h>
#include <config/change_handler.h>

namespace fawkes {
  class TimeWait;
}

class MongoRRDThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::MongoDBAspect,
  public fawkes::RRDAspect,
  public fawkes::ConfigurationChangeHandler
{
 public:
  MongoRRDThread();
  virtual ~MongoRRDThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void add_dbstats(const char *path, std::string dbname);
  void remove_dbstats(const char *path);

  virtual void config_tag_changed(const char *new_tag);
  virtual void config_value_changed(const fawkes::Configuration::ValueIterator *v);
  virtual void config_comment_changed(const fawkes::Configuration::ValueIterator *v);
  virtual void config_value_erased(const char *path);

 private:
  fawkes::TimeWait      *__timewait;

  fawkes::RRDDefinition *__opcounters_rrd;
  fawkes::RRDDefinition *__memory_rrd;
  fawkes::RRDDefinition *__indexes_rrd;
  fawkes::RRDDefinition *__locks_rrd;

  fawkes::RRDGraphDefinition *__opcounters_graph;
  fawkes::RRDGraphDefinition *__memory_graph;
  fawkes::RRDGraphDefinition *__indexes_graph;

  /// @cond INTERNALS
  typedef struct {
    std::string                 db_name;
    std::string                 rrd_name;
    std::string                 conf_path;
    fawkes::RRDDefinition      *rrd;
    fawkes::RRDGraphDefinition *graph1;
    fawkes::RRDGraphDefinition *graph2;
    fawkes::RRDGraphDefinition *graph3;
  } DbStatsInfo;
  /// @endcond

  typedef std::map<std::string, DbStatsInfo> DbStatsMap;
  DbStatsMap __dbstats;
};

#endif
