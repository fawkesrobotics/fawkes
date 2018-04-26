
/***************************************************************************
 *  clips_robot_memory_thread.h - CLIPS feature for accessing the robot memory
 *
 *  Plugin created: Mon Aug 29 15:41:47 2016

 *  Copyright  2016  Frederik Zwilling
 *             2013  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_CLIPS_ROBOT_MEMORYTHREAD_H_
#define __PLUGINS_CLIPS_ROBOT_MEMORYTHREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <plugins/clips/aspect/clips_feature.h>
#include <plugins/robot-memory/aspect/robot_memory_aspect.h>
#include "clips_rm_trigger.h"

#include <string>
#include <future>
#include <clipsmm.h>

namespace fawkes {
}

class ClipsRobotMemoryThread 
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::CLIPSFeature,
  public fawkes::CLIPSFeatureAspect,
  public fawkes::RobotMemoryAspect
{

 public:
  ClipsRobotMemoryThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

  // for CLIPSFeature
  virtual void clips_context_init(const std::string &env_name,
          fawkes::LockPtr<CLIPS::Environment> &clips);
  virtual void clips_context_destroyed(const std::string &env_name);

  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
  protected: virtual void run() { Thread::run(); }

 private:
  std::map<std::string, fawkes::LockPtr<CLIPS::Environment> >  envs_;

  CLIPS::Value  clips_bson_create();
  CLIPS::Value  clips_bson_parse(std::string document);
  void          clips_bson_destroy(void *bson);
  void          clips_bson_append(void *bson, std::string field_name, CLIPS::Value value);
  void          clips_bson_append_array(void *bson,
          std::string field_name, CLIPS::Values values);
  void          clips_bson_append_time(void *bson,
               std::string field_name, CLIPS::Values time);
  CLIPS::Value  clips_bson_array_start(void *bson, std::string field_name);
  void          clips_bson_array_finish(void *barr);
  void          clips_bson_array_append(void *barr, CLIPS::Value value);
  std::string   clips_bson_tostring(void *bson);
  CLIPS::Values clips_bson_field_names(void *bson);
  CLIPS::Value  clips_bson_has_field(void *bson, std::string field_name);
  CLIPS::Value  clips_bson_get(void *bson, std::string field_name);
  CLIPS::Values clips_bson_get_array(void *bson, std::string field_name);
  CLIPS::Values clips_bson_get_time(void *bson, std::string field_name);

  void          clips_robotmemory_upsert(std::string collection, void *bson, CLIPS::Value query);
  void          clips_robotmemory_update(std::string collection, void *bson, CLIPS::Value query);
  void          clips_robotmemory_replace(std::string collection, void *bson, CLIPS::Value query);
  void          clips_robotmemory_insert(std::string collection, void *bson);
  void          clips_robotmemory_create_index(std::string collection, void *bson);
  void          clips_robotmemory_create_unique_index(std::string collection, void *bson);
  void          robotmemory_update(std::string &collection, mongo::BSONObj obj,
                               CLIPS::Value &query, bool upsert);
  CLIPS::Value  clips_robotmemory_query_sort(std::string collection, void *bson, void *bson_sort);
  CLIPS::Value  clips_robotmemory_query(std::string collection, void *bson);
  void          clips_robotmemory_remove(std::string collection, void *bson);
  CLIPS::Value  clips_robotmemory_cursor_more(void *cursor);
  CLIPS::Value  clips_robotmemory_cursor_next(void *cursor);
  void          clips_robotmemory_cursor_destroy(void *cursor);

  CLIPS::Value  clips_robotmemory_mutex_create(std::string name);
  CLIPS::Value  clips_robotmemory_mutex_destroy(std::string name);
  CLIPS::Value  clips_robotmemory_mutex_try_lock(std::string name, std::string identity);
  CLIPS::Value  clips_robotmemory_mutex_renew_lock(std::string name, std::string identity);
  CLIPS::Value  clips_robotmemory_mutex_force_lock(std::string name, std::string identity);
  CLIPS::Value  clips_robotmemory_mutex_unlock(std::string name, std::string identity);
  CLIPS::Value  clips_robotmemory_mutex_setup_ttl(float max_age_sec);
  CLIPS::Value  clips_robotmemory_mutex_expire_locks(float max_age_sec);

  CLIPS::Values clips_robotmemory_mutex_create_async(std::string name);
  CLIPS::Values clips_robotmemory_mutex_destroy_async(std::string name);
  CLIPS::Values clips_robotmemory_mutex_try_lock_async(std::string env_name,
                                                       std::string name, std::string identity);
  CLIPS::Values clips_robotmemory_mutex_renew_lock_async(std::string env_name,
                                                         std::string name, std::string identity);
  CLIPS::Values clips_robotmemory_mutex_force_lock_async(std::string name, std::string identity);
  CLIPS::Values clips_robotmemory_mutex_unlock_async(std::string name, std::string identity);
  CLIPS::Value  clips_robotmemory_mutex_expire_locks_async(std::string env_name, float max_age_sec);

  CLIPS::Value  clips_robotmemory_register_trigger(std::string env_name, std::string collection, void *query, std::string assert_name);
  void  clips_robotmemory_destroy_trigger(void *trigger);

  bool mutex_future_ready(const std::string& name);

 private:
  std::list<ClipsRmTrigger*>  clips_triggers_;
  std::map<std::string, std::future<bool>> mutex_futures_;
  std::future<bool> mutex_expire_future_;
};


#endif
