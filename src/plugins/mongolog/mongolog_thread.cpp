
/***************************************************************************
 *  mongolog_thread.cpp - MongoDB Logging Thread
 *
 *  Created: Wed Dec 08 23:09:29 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#include "mongolog_thread.h"

#include <core/threading/mutex_locker.h>
#include <cstdlib>

// from MongoDB
#include <mongo/client/dbclient.h>

using namespace mongo;
using namespace fawkes;


/** @class MongoLogThread "mongodb_thread.h"
 * MongoDB Logging Thread.
 * This thread registers to interfaces specified with patterns in the
 * configurationa and logs any changes to MongoDB.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
MongoLogThread::MongoLogThread()
  : Thread("MongoLogThread", Thread::OPMODE_WAITFORWAKEUP)
{
}


/** Destructor. */
MongoLogThread::~MongoLogThread()
{
}


void
MongoLogThread::init()
{
  /*
  std::set<std::string> ignored_configs;

  std::string prefix = "/plugins/mongodb/clients/";

  std::auto_ptr<Configuration::ValueIterator> i(config->search(prefix.c_str()));
  while (i->next()) {
    std::string cfg_name = std::string(i->path()).substr(prefix.length());
    cfg_name = cfg_name.substr(0, cfg_name.find("/"));

    if ( (__configs.find(cfg_name) == __configs.end()) &&
	 (ignored_configs.find(cfg_name) == ignored_configs.end()) ) {

      std::string cfg_prefix = prefix + cfg_name + "/";

      try {
	ClientConf *conf = new ClientConf(config, logger, cfg_name, cfg_prefix);
	__configs[cfg_name] = conf;
	logger->log_info(name(), "Added MongoDB client configuration %s",
			 cfg_name.c_str());
      } catch (Exception &e) {
	logger->log_warn(name(), "Invalid MongoDB client config %s, ignoring, "
			 "exception follows.", cfg_name.c_str());
	ignored_configs.insert(cfg_name);
      }
    }
  }

  if (__configs.empty()) {
    throw Exception("No active MongoDB configurations found");
  }
  */

  __database = "fflog";
  try {
    __database = config->get_string("/plugins/mongolog/database");
  } catch (Exception &e) {
    logger->log_info(name(), "No database configured, writing to %s",
		     __database.c_str());
  }

  bbio_add_observed_create("*", "*");

  std::list<Interface *> current_interfaces =
    blackboard->open_multiple_for_reading("*", "*");

  std::list<Interface *>::iterator i;
  for (i = current_interfaces.begin(); i != current_interfaces.end(); ++i) {
    logger->log_debug(name(), "Opening %s", (*i)->uid());
    __listeners[(*i)->uid()] = new InterfaceListener(blackboard, *i,
						     mongodb_client, __database,
						     __collections, logger);
  }

  blackboard->register_observer(this, BlackBoard::BBIO_FLAG_CREATED);

  config->set_string("/plugins/mongorrd/databases/mongolog", __database);
}


void
MongoLogThread::finalize()
{
  /*
  std::map<std::string, ClientConf *>::iterator i;
  for (i = __configs.begin(); i != __configs.end(); ++i) {
    delete i->second;
  }
  __configs.clear();
  */
  config->erase("/plugins/mongorrd/databases/mongolog");
  std::map<std::string, InterfaceListener *>::iterator i;
  for (i = __listeners.begin(); i != __listeners.end(); ++i) {
    delete i->second;
  }
  __listeners.clear();
}


void
MongoLogThread::loop()
{
}

// for BlackBoardInterfaceObserver
void
MongoLogThread::bb_interface_created(const char *type, const char *id) throw()
{
  MutexLocker lock(__listeners.mutex());

  try {
    Interface *interface = blackboard->open_for_reading(type, id);
    if (__listeners.find(interface->uid()) == __listeners.end()) {
      logger->log_debug(name(), "Opening new %s", interface->uid());
      __listeners[interface->uid()] = new InterfaceListener(blackboard, interface,
							    mongodb_client,
							    __database,
							    __collections,
							    logger);
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




MongoLogThread::InterfaceListener::InterfaceListener(BlackBoard *blackboard,
						     Interface *interface,
						     mongo::DBClientBase *mongodb,
						     std::string &database,
						     LockSet<std::string> &colls,
						     Logger *logger)
  : BlackBoardInterfaceListener("MongoLogListener-%s", interface->uid()),
    __database(database), __collections(colls)
{
  __blackboard = blackboard;
  __interface  = interface;
  __mongodb    = mongodb;
  __logger     = logger;

  // sanitize interface ID to be suitable for MongoDB
  std::string id = interface->id();
  size_t pos = 0;
  size_t at;
  while((at = id.find(" ", pos)) != string::npos) {
    id.replace(at, 1, "_");
    pos = pos + 1;
  }
  __collection = __database + "." + interface->type() + "." + id;
  if (__collections.find(__collection) != __collections.end()) {
    throw Exception("Collection named %s already used, cannot log %s",
		    __collection.c_str(), interface->uid());
  }

  bbil_add_data_interface(interface);
  __blackboard->register_listener(this, BlackBoard::BBIL_FLAG_DATA);
}


MongoLogThread::InterfaceListener::~InterfaceListener()
{
  __blackboard->unregister_listener(this);
}

void
MongoLogThread::InterfaceListener::bb_interface_data_changed(Interface *interface) throw()
{
  interface->read();

  // write interface data
  BSONObjBuilder document;
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

  __mongodb->insert(__collection, document.obj());
}
