
/***************************************************************************
 *  clips_robot_memory_thread.cpp - CLIPS feature for accessing the robot memory
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

#include "clips_robot_memory_thread.h"

using namespace fawkes;

/** @class ClipsRobotMemoryThread 'clips_robot_memory_thread.h' 
 * CLIPS feature to access the robot memory
 * @author Frederik Zwilling
 */

ClipsRobotMemoryThread::ClipsRobotMemoryThread()
 : Thread("ClipsRobotMemoryThread", Thread::OPMODE_WAITFORWAKEUP),
   CLIPSFeature("robot_memory"), CLIPSFeatureAspect(this)
{
}

void
ClipsRobotMemoryThread::init()
{
}

void
ClipsRobotMemoryThread::loop()
{
}

void
ClipsRobotMemoryThread::finalize()
{
  envs_.clear();
}

void
ClipsRobotMemoryThread::clips_context_init(const std::string &env_name,
          LockPtr<CLIPS::Environment> &clips)
{
  envs_[env_name] = clips;
  logger->log_debug(name(), "Called to initialize environment %s", env_name.c_str());

  clips.lock();

  clips->add_function("bson-create", sigc::slot<CLIPS::Value>(sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_create)));
  clips->add_function("bson-parse", sigc::slot<CLIPS::Value, std::string>(sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_parse)));
  clips->add_function("bson-destroy", sigc::slot<void, void *>(sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_destroy)));
  clips->add_function("bson-append", sigc::slot<void, void *, std::string, CLIPS::Value>(sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_append)));
  clips->add_function("bson-append-array", sigc::slot<void, void *, std::string, CLIPS::Values>(sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_append_array)));
  clips->add_function("bson-array-start", sigc::slot<CLIPS::Value, void *, std::string>(sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_array_start)));
  clips->add_function("bson-array-finish", sigc::slot<void, void *>(sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_array_finish)));
  clips->add_function("bson-array-append", sigc::slot<void, void *, CLIPS::Value>(sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_array_append)));

  clips->add_function("bson-append-time", sigc::slot<void, void *, std::string, CLIPS::Values>(sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_append_time)));
  clips->add_function("bson-tostring", sigc::slot<std::string, void *>(sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_tostring)));
  clips->add_function("robmem-insert", sigc::slot<void, std::string, void *>(sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_robotmemory_insert)));
  clips->add_function("robmem-upsert", sigc::slot<void, std::string, void *, CLIPS::Value>(sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_robotmemory_upsert)));
  clips->add_function("robmem-update", sigc::slot<void, std::string, void *, CLIPS::Value>(sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_robotmemory_update)));
  clips->add_function("robmem-replace", sigc::slot<void, std::string, void *, CLIPS::Value>(sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_robotmemory_replace)));
  clips->add_function("robmem-query", sigc::slot<CLIPS::Value, std::string, void *>(sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_robotmemory_query)));
  clips->add_function("robmem-query-sort", sigc::slot<CLIPS::Value, std::string, void *, void *>(sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_robotmemory_query_sort)));
  clips->add_function("robmem-cursor-destroy", sigc::slot<void, void *>(sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_robotmemory_cursor_destroy)));
  clips->add_function("robmem-cursor-more", sigc::slot<CLIPS::Value, void *>(sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_robotmemory_cursor_more)));
  clips->add_function("robmem-cursor-next", sigc::slot<CLIPS::Value, void *>(sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_robotmemory_cursor_next)));
  clips->add_function("bson-field-names", sigc::slot<CLIPS::Values, void *>(sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_field_names)));
  clips->add_function("bson-get", sigc::slot<CLIPS::Value, void *, std::string>(sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_get)));
  clips->add_function("bson-get-array", sigc::slot<CLIPS::Values, void *, std::string>(sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_get_array)));
  clips->add_function("bson-get-time", sigc::slot<CLIPS::Values, void *, std::string>(sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_get_time)));

  clips->build("(deffacts have-feature-mongodb (have-feature MongoDB))");

  //TODO: clips->batch_evaluate(SRCDIR"/plugins/clips-robot-memory/robot_memory.clp");

  clips.unlock();
}

void
ClipsRobotMemoryThread::clips_context_destroyed(const std::string &env_name)
{
  envs_.erase(env_name);
  logger->log_debug(name(), "Removing environment %s", env_name.c_str());
}


CLIPS::Value
ClipsRobotMemoryThread::clips_bson_create()
{
  mongo::BSONObjBuilder *b = new mongo::BSONObjBuilder();
  return CLIPS::Value(b);
}

CLIPS::Value
ClipsRobotMemoryThread::clips_bson_parse(std::string document)
{
  mongo::BSONObjBuilder *b = new mongo::BSONObjBuilder();
  try {
    b->appendElements(mongo::fromjson(document));
#ifdef HAVE_MONGODB_VERSION_H
  } catch (mongo::MsgAssertionException &e) {
#else
  } catch (mongo::AssertionException &e) {
#endif
    logger->log_error("MongoDB", "Parsing JSON doc failed: %s\n%s",
           e.what(), document.c_str());
  }
  return CLIPS::Value(b);
}

void
ClipsRobotMemoryThread::clips_bson_destroy(void *bson)
{
  mongo::BSONObjBuilder *b = static_cast<mongo::BSONObjBuilder *>(bson);
  delete b;
}

std::string
ClipsRobotMemoryThread::clips_bson_tostring(void *bson)
{
  mongo::BSONObjBuilder *b = static_cast<mongo::BSONObjBuilder *>(bson);
  return b->asTempObj().jsonString(mongo::Strict, true);
}

void
ClipsRobotMemoryThread::clips_bson_append(void *bson, std::string field_name, CLIPS::Value value)
{
  try {
    mongo::BSONObjBuilder *b = static_cast<mongo::BSONObjBuilder *>(bson);
    switch (value.type()) {
    case CLIPS::TYPE_FLOAT:
      b->append(field_name, value.as_float());
      break;

    case CLIPS::TYPE_INTEGER:
      b->append(field_name, value.as_integer());
      break;

    case CLIPS::TYPE_SYMBOL:
    case CLIPS::TYPE_STRING:
    case CLIPS::TYPE_INSTANCE_NAME:
      b->append(field_name, value.as_string());
      break;

    case CLIPS::TYPE_EXTERNAL_ADDRESS:
      {
  mongo::BSONObjBuilder *subb = static_cast<mongo::BSONObjBuilder *>(value.as_address());
  b->append(field_name, subb->asTempObj());
      }
      break;

    default:
      logger->log_warn("RefBox", "Tried to add unknown type to BSON field %s",
      field_name.c_str());
      break;
    }
#ifdef HAVE_MONGODB_VERSION_H
  } catch (mongo::MsgAssertionException &e) {
#else
  } catch (mongo::AssertionException &e) {
#endif
    logger->log_error("MongoDB", "Failed to append array value to field %s: %s",
           field_name.c_str(), e.what());
  }
}


void
ClipsRobotMemoryThread::clips_bson_append_array(void *bson,
            std::string field_name, CLIPS::Values values)
{
  try {
    mongo::BSONObjBuilder *b = static_cast<mongo::BSONObjBuilder *>(bson);
    mongo::BSONArrayBuilder ab(b->subarrayStart(field_name));

    for (auto value : values) {
      switch (value.type()) {
      case CLIPS::TYPE_FLOAT:
  ab.append(value.as_float());
  break;

      case CLIPS::TYPE_INTEGER:
  ab.append(value.as_integer());
  break;

      case CLIPS::TYPE_SYMBOL:
      case CLIPS::TYPE_STRING:
      case CLIPS::TYPE_INSTANCE_NAME:
  ab.append(value.as_string());
  break;

      case CLIPS::TYPE_EXTERNAL_ADDRESS:
  {
    mongo::BSONObjBuilder *subb =
      static_cast<mongo::BSONObjBuilder *>(value.as_address());
    ab.append(subb->asTempObj());
  }
  break;

      default:
  logger->log_warn("MongoDB", "Tried to add unknown type to BSON array field %s",
        field_name.c_str());
  break;
      }
    }
#ifdef HAVE_MONGODB_VERSION_H
  } catch (mongo::MsgAssertionException &e) {
#else
  } catch (mongo::AssertionException &e) {
#endif
    logger->log_error("MongoDB", "Failed to append array value to field %s: %s",
           field_name.c_str(), e.what());
  }
}

CLIPS::Value
ClipsRobotMemoryThread::clips_bson_array_start(void *bson, std::string field_name)
{
  mongo::BSONObjBuilder *b = static_cast<mongo::BSONObjBuilder *>(bson);
  mongo::BufBuilder &bb = b->subarrayStart(field_name);
  mongo::BSONArrayBuilder *arrb = new mongo::BSONArrayBuilder(bb);
  return CLIPS::Value(arrb);
}


void
ClipsRobotMemoryThread::clips_bson_array_finish(void *barr)
{
  mongo::BSONArrayBuilder *ab = static_cast<mongo::BSONArrayBuilder *>(barr);
  delete ab;
}

void
ClipsRobotMemoryThread::clips_bson_array_append(void *barr, CLIPS::Value value)
{
  try {
    mongo::BSONArrayBuilder *ab = static_cast<mongo::BSONArrayBuilder *>(barr);
    switch (value.type()) {
    case CLIPS::TYPE_FLOAT:
      ab->append(value.as_float());
      break;

    case CLIPS::TYPE_INTEGER:
      ab->append(value.as_integer());
      break;

    case CLIPS::TYPE_SYMBOL:
    case CLIPS::TYPE_STRING:
    case CLIPS::TYPE_INSTANCE_NAME:
      ab->append(value.as_string());
      break;

    case CLIPS::TYPE_EXTERNAL_ADDRESS:
      {
  mongo::BSONObjBuilder *subb = static_cast<mongo::BSONObjBuilder *>(value.as_address());
  ab->append(subb->asTempObj());
      }
      break;

    default:
      logger->log_warn("RefBox", "Tried to add unknown type to BSON array");
      break;
    }
#ifdef HAVE_MONGODB_VERSION_H
  } catch (mongo::MsgAssertionException &e) {
#else
  } catch (mongo::AssertionException &e) {
#endif
    logger->log_error("MongoDB", "Failed to append to array: %s", e.what());
  }
}


void
ClipsRobotMemoryThread::clips_bson_append_time(void *bson, std::string field_name, CLIPS::Values time)
{
  if (time.size() != 2) {
    logger->log_warn("MongoDB", "Invalid time, %zu instead of 2 entries", time.size());
    return;
  }
  if (time[0].type() != CLIPS::TYPE_INTEGER || time[1].type() != CLIPS::TYPE_INTEGER) {
    logger->log_warn("MongoDB", "Invalid time, type mismatch");
    return;
  }

  try {
    mongo::BSONObjBuilder *b = static_cast<mongo::BSONObjBuilder *>(bson);
    struct timeval now = { time[0].as_integer(), time[1].as_integer()};
    mongo::Date_t nowd = now.tv_sec * 1000 + now.tv_usec / 1000;
    b->appendDate(field_name, nowd);
#ifdef HAVE_MONGODB_VERSION_H
  } catch (mongo::MsgAssertionException &e) {
#else
  } catch (mongo::AssertionException &e) {
#endif
    logger->log_error("MongoDB", "Failed to append time value to field %s: %s",
           field_name.c_str(), e.what());
  }
}

void
ClipsRobotMemoryThread::clips_robotmemory_insert(std::string collection, void *bson)
{
  mongo::BSONObjBuilder *b = static_cast<mongo::BSONObjBuilder *>(bson);

  try {
    robot_memory->insert(b->asTempObj(), collection);
  } catch (mongo::DBException &e) {
    logger->log_warn("MongoDB", "Insert failed: %s", e.what());
  }
}


void
ClipsRobotMemoryThread::robotmemory_update(std::string &collection, mongo::BSONObj obj,
         CLIPS::Value &query, bool upsert)
{
  try {
    mongo::BSONObj query_obj;
    if (query.type() == CLIPS::TYPE_STRING) {
      query_obj = mongo::fromjson(query.as_string());
    } else if (query.type() == CLIPS::TYPE_EXTERNAL_ADDRESS) {
      mongo::BSONObjBuilder *qb = static_cast<mongo::BSONObjBuilder *>(query.as_address());
      query_obj = qb->asTempObj();
    } else {
      logger->log_warn("MongoDB", "Invalid query, must be string or BSON document");
      return;
    }

    robot_memory->update(query_obj, obj, collection, upsert);
#ifdef HAVE_MONGODB_VERSION_H
  } catch (mongo::MsgAssertionException &e) {
#else
  } catch (mongo::AssertionException &e) {
#endif
    logger->log_warn("MongoDB", "Compiling query failed: %s", e.what());
  } catch (mongo::DBException &e) {
    logger->log_warn("MongoDB", "Insert failed: %s", e.what());
  }
}


void
ClipsRobotMemoryThread::clips_robotmemory_upsert(std::string collection, void *bson, CLIPS::Value query)
{
  mongo::BSONObjBuilder *b = static_cast<mongo::BSONObjBuilder *>(bson);
  if (! b) {
    logger->log_warn("MongoDB", "Invalid BSON Obj Builder passed");
    return;
  }
  robotmemory_update(collection, b->asTempObj(), query, true);
}

void
ClipsRobotMemoryThread::clips_robotmemory_update(std::string collection, void *bson, CLIPS::Value query)
{
  mongo::BSONObjBuilder *b = static_cast<mongo::BSONObjBuilder *>(bson);
  if (! b) {
    logger->log_warn("MongoDB", "Invalid BSON Obj Builder passed");
    return;
  }

  mongo::BSONObjBuilder update_doc;
  update_doc.append("$set", b->asTempObj());

  robotmemory_update(collection, update_doc.obj(), query, false);
}

void
ClipsRobotMemoryThread::clips_robotmemory_replace(std::string collection, void *bson, CLIPS::Value query)
{
  mongo::BSONObjBuilder *b = static_cast<mongo::BSONObjBuilder *>(bson);
  if (! b) logger->log_warn("MongoDB", "Invalid BSON Obj Builder passed");
  robotmemory_update(collection, b->asTempObj(), query, false);
}

CLIPS::Value
ClipsRobotMemoryThread::clips_robotmemory_query_sort(std::string collection, void *bson, void *bson_sort)
{
  mongo::BSONObjBuilder *b = static_cast<mongo::BSONObjBuilder *>(bson);

  try {
    mongo::Query q(b->asTempObj());
    if (bson_sort) {
      mongo::BSONObjBuilder *bs = static_cast<mongo::BSONObjBuilder *>(bson_sort);
      q.sort(bs->asTempObj());
    }

#if __cplusplus >= 201103L
    std::unique_ptr<mongo::DBClientCursor> c = robot_memory->query(q, collection);

    return CLIPS::Value(new std::unique_ptr<mongo::DBClientCursor>(std::move(c)),
                        CLIPS::TYPE_EXTERNAL_ADDRESS);
#else
    std::auto_ptr<mongo::DBClientCursor> c = mongodb_->query(collection, q);

    return CLIPS::Value(new std::auto_ptr<mongo::DBClientCursor>(c),
                        CLIPS::TYPE_EXTERNAL_ADDRESS);
#endif

  } catch (mongo::DBException &e) {
    logger->log_warn("MongoDB", "Query failed: %s", e.what());
    return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
  }
}

CLIPS::Value
ClipsRobotMemoryThread::clips_robotmemory_query(std::string collection, void *bson)
{
  return clips_robotmemory_query_sort(collection, bson, NULL);
}

void
ClipsRobotMemoryThread::clips_robotmemory_cursor_destroy(void *cursor)
{
#if __cplusplus >= 201103L
  std::unique_ptr<mongo::DBClientCursor> *c =
    static_cast<std::unique_ptr<mongo::DBClientCursor> *>(cursor);
#else
  std::auto_ptr<mongo::DBClientCursor> *c =
    static_cast<std::auto_ptr<mongo::DBClientCursor> *>(cursor);
#endif

  if (! c || ! c->get()) {
    logger->log_error("MongoDB", "mongodb-cursor-destroy: got invalid cursor");
    return;
  }

  delete c;
}

CLIPS::Value
ClipsRobotMemoryThread::clips_robotmemory_cursor_more(void *cursor)
{
#if __cplusplus >= 201103L
  std::unique_ptr<mongo::DBClientCursor> *c =
    static_cast<std::unique_ptr<mongo::DBClientCursor> *>(cursor);
#else
  std::auto_ptr<mongo::DBClientCursor> *c =
    static_cast<std::auto_ptr<mongo::DBClientCursor> *>(cursor);
#endif

  if (! c || ! c->get()) {
    logger->log_error("MongoDB", "mongodb-cursor-more: got invalid cursor");
    return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
  }

  return CLIPS::Value((*c)->more() ? "TRUE" : "FALSE", CLIPS::TYPE_SYMBOL);
}

CLIPS::Value
ClipsRobotMemoryThread::clips_robotmemory_cursor_next(void *cursor)
{
#if __cplusplus >= 201103L
  std::unique_ptr<mongo::DBClientCursor> *c =
    static_cast<std::unique_ptr<mongo::DBClientCursor> *>(cursor);
#else
  std::auto_ptr<mongo::DBClientCursor> *c =
    static_cast<std::auto_ptr<mongo::DBClientCursor> *>(cursor);
#endif

  if (! c || ! c->get()) {
    logger->log_error("MongoDB", "mongodb-cursor-next: got invalid cursor");
    return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
  }

  mongo::BSONObjBuilder *b = new mongo::BSONObjBuilder();
  b->appendElements((*c)->next());
  return CLIPS::Value(b);
}


CLIPS::Values
ClipsRobotMemoryThread::clips_bson_field_names(void *bson)
{
  mongo::BSONObjBuilder *b = static_cast<mongo::BSONObjBuilder *>(bson);

  if (! b) {
    logger->log_error("MongoDB", "mongodb-bson-field-names: invalid object");
    CLIPS::Values rv;
    rv.push_back(CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
    return rv;
  }

  std::set<std::string> field_names;
  b->asTempObj().getFieldNames(field_names);

  CLIPS::Values rv;
  for (const std::string &n : field_names) {
    rv.push_back(CLIPS::Value(n));
  }
  return rv;
}


CLIPS::Value
ClipsRobotMemoryThread::clips_bson_get(void *bson, std::string field_name)
{
  mongo::BSONObjBuilder *b = static_cast<mongo::BSONObjBuilder *>(bson);

  if (! b) {
    logger->log_error("MongoDB", "mongodb-bson-get: invalid object");
    return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
  }

  mongo::BSONObj o(b->asTempObj());

  if (! o.hasField(field_name)) {
    logger->log_error("MongoDB", "mongodb-bson-get: has no field %s",
                       field_name.c_str());
    return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
  }

  mongo::BSONElement el = o.getField(field_name);

  switch (el.type()) {
  case mongo::NumberDouble:
    return CLIPS::Value(el.Double());
  case mongo::String:
    return CLIPS::Value(el.String());
  case mongo::Bool:
    return CLIPS::Value(el.Bool() ? "TRUE" : "FALSE", CLIPS::TYPE_SYMBOL);
  case mongo::NumberInt:
    return CLIPS::Value(el.Int());
  case mongo::NumberLong:
    return CLIPS::Value(el.Long());
  case mongo::Object:
    {
      mongo::BSONObjBuilder *b = new mongo::BSONObjBuilder();
      b->appendElements(o);
      return CLIPS::Value(b);
    }
  default:
    return CLIPS::Value("INVALID_VALUE_TYPE", CLIPS::TYPE_SYMBOL);
  }
}


CLIPS::Values
ClipsRobotMemoryThread::clips_bson_get_array(void *bson, std::string field_name)
{
  mongo::BSONObjBuilder *b = static_cast<mongo::BSONObjBuilder *>(bson);

  CLIPS::Values rv;

  if (! b) {
    logger->log_error("MongoDB", "mongodb-bson-get-array: invalid object");
    rv.push_back(CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
    return rv;
  }

  mongo::BSONObj o(b->asTempObj());

  if (! o.hasField(field_name)) {
    logger->log_error("MongoDB", "mongodb-bson-get-array: has no field %s",
                       field_name.c_str());
    rv.push_back(CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
    return rv;
  }

  mongo::BSONElement el = o.getField(field_name);

  if (el.type() != mongo::Array) {
    logger->log_error("MongoDB", "mongodb-bson-get-array: field %s is not an array",
                       field_name.c_str());
    rv.push_back(CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
    return rv;
  }

  std::vector<mongo::BSONElement> elements(el.Array());

  for (const mongo::BSONElement &e : elements) {
    switch (e.type()) {
    case mongo::NumberDouble:
      rv.push_back(CLIPS::Value(e.Double())); break;
    case mongo::String:
      rv.push_back(CLIPS::Value(e.String())); break;
    case mongo::Bool:
      rv.push_back(CLIPS::Value(e.Bool() ? "TRUE" : "FALSE", CLIPS::TYPE_SYMBOL));
      break;
    case mongo::NumberInt:
      rv.push_back(CLIPS::Value(e.Int())); break;
    case mongo::NumberLong:
      rv.push_back(CLIPS::Value(e.Long())); break;
    case mongo::Object:
      {
        mongo::BSONObjBuilder *b = new mongo::BSONObjBuilder();
        b->appendElements(e.Obj());
        rv.push_back(CLIPS::Value(b));
      }
      break;
    default:
      rv.clear();
      rv.push_back(CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
      return rv;
    }
  }
  return rv;

}


CLIPS::Values
ClipsRobotMemoryThread::clips_bson_get_time(void *bson, std::string field_name)
{
  mongo::BSONObjBuilder *b = static_cast<mongo::BSONObjBuilder *>(bson);

  CLIPS::Values rv;

  if (! b) {
    logger->log_error("MongoDB", "mongodb-bson-get-time: invalid object");
    rv.push_back(CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
    return rv;
  }

  mongo::BSONObj o(b->asTempObj());

  if (! o.hasField(field_name)) {
    logger->log_error("MongoDB", "mongodb-bson-get-time: has no field %s",
                       field_name.c_str());
    rv.push_back(CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
    return rv;
  }

  mongo::BSONElement el = o.getField(field_name);

  int64_t ts = 0;
  if (el.type() == mongo::Date) {
    mongo::Date_t d = el.Date();
    ts = d.asInt64();
  } else if (el.type() == mongo::Timestamp) {
    mongo::Timestamp_t t = el.Timestamp();
    ts = t.seconds();
  } else {
    logger->log_error("MongoDB", "mongodb-bson-get-time: field %s is not a time",
                       field_name.c_str());
    rv.push_back(CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
    return rv;
  }


  rv.resize(2);
  rv[0] = CLIPS::Value((long long int)(ts / 1000));
  rv[1] = CLIPS::Value((ts - (rv[0].as_integer() * 1000)) * 1000);
  return rv;
}

