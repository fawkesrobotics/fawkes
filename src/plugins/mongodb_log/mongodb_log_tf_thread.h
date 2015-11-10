
/***************************************************************************
 *  mongodb_log_tf_thread.h - MongoDB transforms logging thread
 *
 *  Created: Tue Dec 11 14:55:53 2012 (Freiburg)
 *  Copyright  2010-2012  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_MONGODB_LOG_MONGODB_LOG_TF_THREAD_H_
#define __PLUGINS_MONGODB_LOG_MONGODB_LOG_TF_THREAD_H_

#include <core/threading/thread.h>
#include <utils/time/time.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <aspect/blackboard.h>
#include <aspect/tf.h>
#include <plugins/mongodb/aspect/mongodb.h>

#include <string>
#include <vector>

namespace fawkes {
  class TimeWait;
}

class MongoLogTransformsThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::MongoDBAspect,
  public fawkes::TransformAspect
{
 public:
  MongoLogTransformsThread();
  virtual ~MongoLogTransformsThread();

  virtual void init();
  virtual bool prepare_finalize_user();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void store(std::vector<fawkes::tf::TimeCacheInterfacePtr> &caches,
	     std::vector<fawkes::Time> &from, std::vector<fawkes::Time> &to);

 private:
  fawkes::Mutex    *mutex_;
  fawkes::TimeWait *wait_;
  std::string      database_;
  std::string      collection_;
  float            cfg_storage_interval_;
  std::vector<fawkes::Time> last_tf_range_end_;
};

#endif
