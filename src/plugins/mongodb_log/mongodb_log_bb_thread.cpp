
/***************************************************************************
 *  mongodb_log_bb_thread.cpp - MongoDB blackboard logging Thread
 *
 *  Created: Wed Dec 08 23:09:29 2010
 *  Copyright  2010-2017  Tim Niemueller [www.niemueller.de]
 *             2012       Bastian Klingen
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

#include "mongodb_log_bb_thread.h"

#include <core/threading/mutex_locker.h>
#include <plugins/mongodb/aspect/mongodb_conncreator.h>
#include <cstdlib>

// from MongoDB
#include <mongo/client/dbclient.h>

#include <fnmatch.h>

using namespace mongo;
using namespace fawkes;


/** @class MongoLogBlackboardThread "mongodb_thread.h"
 * MongoDB Logging Thread.
 * This thread registers to interfaces specified with patterns in the
 * configurationa and logs any changes to MongoDB.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
MongoLogBlackboardThread::MongoLogBlackboardThread()
  : Thread("MongoLogBlackboardThread", Thread::OPMODE_WAITFORWAKEUP),
    MongoDBAspect("default")
{
}


/** Destructor. */
MongoLogBlackboardThread::~MongoLogBlackboardThread()
{
}


void
MongoLogBlackboardThread::init()
{
  now_ = new Time(clock);
  database_ = "fflog";
  try {
    database_ = config->get_string("/plugins/mongodb-log/database");
  } catch (Exception &e) {
    logger->log_info(name(), "No database configured, writing to %s",
		     database_.c_str());
  }

  std::vector<std::string> includes;
  try {
    includes = config->get_strings("/plugins/mongodb-log/blackboard/includes");
  } catch (Exception &e) {} // ignored, no include rules
  try {
    excludes_ = config->get_strings("/plugins/mongodb-log/blackboard/excludes");
  } catch (Exception &e) {} // ignored, no include rules

  if (includes.empty()) {
    includes.push_back("*");
  }

  std::vector<std::string>::iterator i;
  std::vector<std::string>::iterator e;
  for (i = includes.begin(); i != includes.end(); ++i) {
    bbio_add_observed_create("*", i->c_str());

    std::list<Interface *> current_interfaces =
      blackboard->open_multiple_for_reading("*", i->c_str());
    
    std::list<Interface *>::iterator i;
    for (i = current_interfaces.begin(); i != current_interfaces.end(); ++i) {
      bool exclude = false;
      for (e = excludes_.begin(); e != excludes_.end(); ++e) {
	if (fnmatch(e->c_str(), (*i)->id(), 0) != FNM_NOMATCH) {
	  logger->log_debug(name(), "Excluding '%s' by config rule", (*i)->uid());
	  blackboard->close(*i);
	  exclude = true;
	  break;
	}
      }
      if (exclude) continue;

      logger->log_debug(name(), "Adding %s", (*i)->uid());
      mongo::DBClientBase *mc = mongodb_connmgr->create_client();
      listeners_[(*i)->uid()] = new InterfaceListener(blackboard, *i, mc, database_,
						      collections_, logger, now_);
    }
  }

  blackboard->register_observer(this);
}


void
MongoLogBlackboardThread::finalize()
{
  blackboard->unregister_observer(this);

  std::map<std::string, InterfaceListener *>::iterator i;
  for (i = listeners_.begin(); i != listeners_.end(); ++i) {
    mongo::DBClientBase *mc = i->second->mongodb_client();
    delete i->second;
    mongodb_connmgr->delete_client(mc);
  }
  listeners_.clear();
}


void
MongoLogBlackboardThread::loop()
{
}

// for BlackBoardInterfaceObserver
void
MongoLogBlackboardThread::bb_interface_created(const char *type, const char *id) throw()
{
  MutexLocker lock(listeners_.mutex());

  std::vector<std::string>::iterator e;
  for (e = excludes_.begin(); e != excludes_.end(); ++e) {
    if (fnmatch(e->c_str(), id, 0) != FNM_NOMATCH) {
      logger->log_debug(name(), "Ignoring excluded interface '%s::%s'", type, id);
      return;
    }
  }

  try {
    Interface *interface = blackboard->open_for_reading(type, id);
    if (listeners_.find(interface->uid()) == listeners_.end()) {
      logger->log_debug(name(), "Opening new %s", interface->uid());
      mongo::DBClientBase *mc = mongodb_connmgr->create_client();
      listeners_[interface->uid()] = new InterfaceListener(blackboard, interface, mc,
							   database_, collections_,
							   logger, now_);
    } else {
      logger->log_warn(name(), "Interface %s already opened", interface->uid());
      blackboard->close(interface);
    }
  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to open interface %s::%s, exception follows",
		     type, id);
    logger->log_warn(name(), e);
  }
}




/** Constructor.
 * @param blackboard blackboard
 * @param interface interface to listen for
 * @param mongodb MongoDB client to write to
 * @param database name of database to write to
 * @param colls collections
 * @param logger logger
 * @param now Time
 */
MongoLogBlackboardThread::InterfaceListener::InterfaceListener(BlackBoard *blackboard,
							       Interface *interface,
							       mongo::DBClientBase *mongodb,
							       std::string &database,
							       LockSet<std::string> &colls,
							       Logger *logger, Time *now)
  : BlackBoardInterfaceListener("MongoLogListener-%s", interface->uid()),
    database_(database), collections_(colls)
{
  blackboard_ = blackboard;
  interface_  = interface;
  mongodb_    = mongodb;
  logger_     = logger;
  now_        = now;

  // sanitize interface ID to be suitable for MongoDB
  std::string id = interface->id();
  size_t pos = 0;
  while((pos = id.find_first_of(" -", pos)) != std::string::npos) {
    id.replace(pos, 1, "_");
    pos = pos + 1;
  }
  collection_ = database_ + "." + interface->type() + "." + id;
  if (collections_.find(collection_) != collections_.end()) {
    throw Exception("Collection named %s already used, cannot log %s",
		    collection_.c_str(), interface->uid());
  }

  bbil_add_data_interface(interface);
  blackboard_->register_listener(this, BlackBoard::BBIL_FLAG_DATA);
}


/** Destructor. */
MongoLogBlackboardThread::InterfaceListener::~InterfaceListener()
{
  blackboard_->unregister_listener(this);
}

void
MongoLogBlackboardThread::InterfaceListener::bb_interface_data_changed(Interface *interface)
  throw()
{
  now_->stamp();
  interface->read();

  try {
    // write interface data
    BSONObjBuilder document;
    document.append("timestamp", (long long) now_->in_msec());
    InterfaceFieldIterator i;
    for (i = interface->fields(); i != interface->fields_end(); ++i) {
      size_t length = i.get_length();
      bool is_array = (length > 1);

      switch (i.get_type()) {
      case IFT_BOOL:
	if (is_array) {
	  bool *bools = i.get_bools();
	  BSONArrayBuilder subb(document.subarrayStart(i.get_name()));
	  for (size_t l = 0; l < length; ++l) {
	    subb.append(bools[l]);
	  }
	  subb.doneFast();
	} else {
	  document.append(i.get_name(), i.get_bool());
	}
	break;

      case IFT_INT8:
	if (is_array) {
	  int8_t *ints = i.get_int8s();
	  BSONArrayBuilder subb(document.subarrayStart(i.get_name()));
	  for (size_t l = 0; l < length; ++l) {
	    subb.append(ints[l]);
	  }
	  subb.doneFast();
	} else {
	  document.append(i.get_name(), i.get_int8());
	}
	break;

      case IFT_UINT8:
	if (is_array) {
	  uint8_t *ints = i.get_uint8s();
	  BSONArrayBuilder subb(document.subarrayStart(i.get_name()));
	  for (size_t l = 0; l < length; ++l) {
	    subb.append(ints[l]);
	  }
	  subb.doneFast();
	} else {
	  document.append(i.get_name(), i.get_uint8());
	}
	break;

      case IFT_INT16:
	if (is_array) {
	  int16_t *ints = i.get_int16s();
	  BSONArrayBuilder subb(document.subarrayStart(i.get_name()));
	  for (size_t l = 0; l < length; ++l) {
	    subb.append(ints[l]);
	  }
	  subb.doneFast();
	} else {
	  document.append(i.get_name(), i.get_int16());
	}
	break;

      case IFT_UINT16:
	if (is_array) {
	  uint16_t *ints = i.get_uint16s();
	  BSONArrayBuilder subb(document.subarrayStart(i.get_name()));
	  for (size_t l = 0; l < length; ++l) {
	    subb.append(ints[l]);
	  }
	  subb.doneFast();
	} else {
	  document.append(i.get_name(), i.get_uint16());
	}
	break;

      case IFT_INT32:
	if (is_array) {
	  int32_t *ints = i.get_int32s();
	  BSONArrayBuilder subb(document.subarrayStart(i.get_name()));
	  for (size_t l = 0; l < length; ++l) {
	    subb.append(ints[l]);
	  }
	  subb.doneFast();
	} else {
	  document.append(i.get_name(), i.get_int32());
	}
	break;

      case IFT_UINT32:
	if (is_array) {
	  uint32_t *ints = i.get_uint32s();
	  BSONArrayBuilder subb(document.subarrayStart(i.get_name()));
	  for (size_t l = 0; l < length; ++l) {
	    subb.append(ints[l]);
	  }
	  subb.doneFast();
	} else {
	  document.append(i.get_name(), i.get_uint32());
	}
	break;

      case IFT_INT64:
	if (is_array) {
	  int64_t *ints = i.get_int64s();
	  BSONArrayBuilder subb(document.subarrayStart(i.get_name()));
	  for (size_t l = 0; l < length; ++l) {
	    subb.append((long long int)ints[l]);
	  }
	  subb.doneFast();
	} else {
	  document.append(i.get_name(), (long long int)i.get_int64());
	}
	break;

      case IFT_UINT64:
	if (is_array) {
	  uint64_t *ints = i.get_uint64s();
	  BSONArrayBuilder subb(document.subarrayStart(i.get_name()));
	  for (size_t l = 0; l < length; ++l) {
	    subb.append((long long int)ints[l]);
	  }
	  subb.doneFast();
	} else {
	  document.append(i.get_name(), (long long int)i.get_uint64());
	}
	break;

      case IFT_FLOAT:
	if (is_array) {
	  float *floats = i.get_floats();
	  BSONArrayBuilder subb(document.subarrayStart(i.get_name()));
	  for (size_t l = 0; l < length; ++l) {
	    subb.append(floats[l]);
	  }
	  subb.doneFast();
	} else {
	  document.append(i.get_name(), i.get_float());
	}
	break;

      case IFT_DOUBLE:
	if (is_array) {
	  double *doubles = i.get_doubles();
	  BSONArrayBuilder subb(document.subarrayStart(i.get_name()));
	  for (size_t l = 0; l < length; ++l) {
	    subb.append(doubles[l]);
	  }
	  subb.doneFast();
	} else {
	  document.append(i.get_name(), i.get_double());
	}
	break;

      case IFT_STRING:
	document.append(i.get_name(), i.get_string());
	break;

      case IFT_BYTE:
	if (is_array) {
	  document.appendBinData(i.get_name(), length,
				 BinDataGeneral, i.get_bytes());
	} else {
	  document.append(i.get_name(), i.get_byte());
	}
	break;

      case IFT_ENUM:
	if (is_array) {
	  int32_t *ints = i.get_enums();
	  BSONArrayBuilder subb(document.subarrayStart(i.get_name()));
	  for (size_t l = 0; l < length; ++l) {
	    subb.append(ints[l]);
	  }
	  subb.doneFast();
	} else {
	  document.append(i.get_name(), i.get_enum());
	}
	break;
      }
    }

    mongodb_->insert(collection_, document.obj());
  } catch (mongo::DBException &e) {
    logger_->log_warn(bbil_name(), "Failed to log to %s: %s",
                       collection_.c_str(), e.what());
  } catch (std::exception &e) {
    logger_->log_warn(bbil_name(), "Failed to log to %s: %s (*)",
                       collection_.c_str(), e.what());
  }
}
