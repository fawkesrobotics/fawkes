
/***************************************************************************
 *  netconf.cpp - Fawkes remote configuration access via Fawkes net
 *
 *  Created: Sun Jan 07 15:04:41 2007
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <config/netconf.h>
#include <config/net_messages.h>
#include <config/memory.h>
#include <config/net_list_content.h>

#include <core/threading/mutex.h>
#include <core/threading/interruptible_barrier.h>
#include <netcomm/fawkes/client.h>
#include <netcomm/fawkes/message.h>
#include <netcomm/utils/exceptions.h>

#include <logging/liblogger.h>
#include <utils/misc/string_conversions.h>

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class CannotEnableMirroringException <config/netconf.h>
 * Thrown if enabling mirror mode failed.
 */

/** Constructor.
 * @param msg message describing the problem
 */
CannotEnableMirroringException::CannotEnableMirroringException(const char *msg)
  : Exception("Could not enable mirroring: %s", msg)
{
}


/** @class NetworkConfiguration <config/netconf.h>
 * Remote configuration via Fawkes net.
 * This implementation of the Configuration interface allows for remote access
 * to a Fawkes process implemented using the ConfigurationManager.
 *
 * The network configuration can operator in two modes. In mirror and in non-mirror
 * mode. The non-mirror mode is recommended if only a few operations have to be
 * carried out like getting only a very few values or setting a single value.
 * The mirror mode is for longer usage periods and on-the-fly updates. In mirror
 * mode the complete configuration is copied once from the Fawkes process and then
 * all updates are incorporated into the local database. You can register change
 * handlers to be notified as soon as someone modifies a value.
 *
 */

/** Constructor.
 * @param c Fawkes network client (thread).
 * @param mirror_timeout_sec timeout in seconds for initiating mirroring
 */
NetworkConfiguration::NetworkConfiguration(FawkesNetworkClient *c,
					   unsigned int mirror_timeout_sec)
{
  __mirror_timeout_sec = mirror_timeout_sec;
  __connected = c->connected();
  this->c = c;
  try {
    c->register_handler(this, FAWKES_CID_CONFIGMANAGER);
  } catch (Exception &e) {
    e.append("Failed to register for config manager component on network client");
    throw;
  }
  mutex = new Mutex();
  msg = NULL;
  __mirror_mode = false;
  __mirror_mode_before_connection_dead = false;
  __mirror_init_barrier = NULL;
}


/** Destructor. */
NetworkConfiguration::~NetworkConfiguration()
{
  set_mirror_mode(false);
  c->deregister_handler(FAWKES_CID_CONFIGMANAGER);
  if (msg != NULL) {
    msg->unref();
  }
  delete __mirror_init_barrier;
  delete mutex;
}


void
NetworkConfiguration::load(const char *file_path)
{
}


/** Copy all values from the given configuration.
 * All values from the given configuration are copied. Old values are not erased
 * so that the copied values will overwrite existing values, new values are
 * created, but values existent in current config but not in the copie config
 * will remain unchanged.
 * @param copyconf configuration to copy
 */
void
NetworkConfiguration::copy(Configuration *copyconf)
{
  copyconf->lock();
  Configuration::ValueIterator *i = copyconf->iterator();
  while ( i->next() ) {
    if ( i->is_float() ) {
      set_float(i->path(), i->get_float());
    } else if ( i->is_int() ) {
      set_int(i->path(), i->get_int());
    } else if ( i->is_uint() ) {
      set_uint(i->path(), i->get_uint());
    } else if ( i->is_bool() ) {
      set_bool(i->path(), i->get_bool());
    } else if ( i->is_string() ) {
      std::string s = i->get_string();
      set_string(i->path(), s);
    }
  }
  delete i;
  copyconf->unlock();
}

bool
NetworkConfiguration::exists(const char *path)
{
  ValueIterator *i = get_value(path);
  bool rv = i->valid();
  delete i;
  return rv;
}


bool
NetworkConfiguration::is_default(const char *path)
{
  ValueIterator *i = get_value(path);
  bool rv = i->is_default();
  delete i;
  return rv;
}


/** Get type of field.
 * @param path path
 * @return string of type
 */
std::string
NetworkConfiguration::get_type(const char *path)
{
  std::string s = "";
  mutex->lock();
  if ( __mirror_mode ) {
    s = mirror_config->get_type(path);
    mutex->unlock();
  } else {
    mutex->unlock();
    Configuration::ValueIterator *i = get_value(path);
    s = i->type();
    delete i;
  }
  return s;
}


bool
NetworkConfiguration::is_float(const char *path)
{
  return (get_type(path) == "float");
}


bool
NetworkConfiguration::is_uint(const char *path)
{
  return (get_type(path) == "unsigned int");
}


bool
NetworkConfiguration::is_int(const char *path)
{
  return (get_type(path) == "int");
}


bool
NetworkConfiguration::is_bool(const char *path)
{
  return (get_type(path) == "bool");
}


bool
NetworkConfiguration::is_string(const char *path)
{
  return (get_type(path) == "string");
}

bool
NetworkConfiguration::is_list(const char *path)
{
  return false;
}

void
NetworkConfiguration::send_get(const char *path, unsigned int msgid)
{
  if ( ! __connected ) {
    throw ConnectionDiedException("NetworkConfiguration: Cannot send get, "
				  "client connection is not alive");
  }
  config_getval_msg_t *g = (config_getval_msg_t *)calloc(1, sizeof(config_getval_msg_t));
  strncpy(g->cp.path, path, CONFIG_MSG_PATH_LENGTH);
  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_CONFIGMANAGER,
							msgid,
							g, sizeof(config_getval_msg_t));
  c->enqueue_and_wait(omsg);

  if ( msg == NULL ) {
    mutex->unlock();
    throw NullPointerException("NetworkConfiguration::send_get: msg == NULL");
  }

  if ( msg->msgid() != msgid ) {
    msg->unref();
    msg = NULL;
    mutex->unlock();
    throw TypeMismatchException("NetworkConfiguration::send_get: msg type not float");
  }
}


float
NetworkConfiguration::get_float(const char *path)
{
  if ( strlen(path) > CONFIG_MSG_PATH_LENGTH ) {
    throw OutOfBoundsException("NetworkConfiguration::get_float: "
			       "Maximum length for path exceeded");
  }
  if ( ! __connected ) {
    throw ConnectionDiedException("NetworkConfiguration: Cannot send get, "
				  "client connection is not alive");
  }

  float f;
  mutex->lock();

  if ( __mirror_mode ) {
    try {
      f = mirror_config->get_float(path);
    } catch (Exception &e) {
      e.append("NetworkConfiguration[mirroring]::get_float: exception in mirror database");
      mutex->unlock();
      throw;
    }
  } else {
    try {
      send_get(path, MSG_CONFIG_GET_FLOAT);

      config_descriptor_t *d = msg->msgge<config_descriptor_t>();
      if (d->num_values > 0) {
	msg->unref();
	msg = NULL;
	throw Exception("NetworkConfiguration: received list of and not a single value");
      }
      f = *(float *)((char *)msg->payload() + sizeof(config_descriptor_t));

      msg->unref();
      msg = NULL;

    } catch (Exception &e) {
      e.append("NetworkConfiguration::get_float: Fetching float failed");
      if ( msg != NULL ) {
	msg->unref();
	msg = NULL;
      }
      mutex->unlock();
      throw;
    }
  }

  mutex->unlock();

  return f;
}


unsigned int
NetworkConfiguration::get_uint(const char *path)
{
  if ( strlen(path) > CONFIG_MSG_PATH_LENGTH ) {
    throw OutOfBoundsException("NetworkConfiguration::get_uint: "
			       "Maximum length for path exceeded");
  }
  if ( ! __connected ) {
    throw ConnectionDiedException("NetworkConfiguration: Cannot send get, "
				  "client connection is not alive");
  }

  unsigned int u;
  mutex->lock();

  if ( __mirror_mode ) {
    try {
      u = mirror_config->get_uint(path);
    } catch (Exception &e) {
      e.append("NetworkConfiguration[mirroring]::get_uint: exception in mirror database");
      mutex->unlock();
      throw;
    }
  } else {
    try {
      send_get(path, MSG_CONFIG_GET_UINT);

      config_descriptor_t *d = msg->msgge<config_descriptor_t>();
      if (d->num_values > 0) {
	msg->unref();
	msg = NULL;
	throw Exception("NetworkConfiguration: received list of and not a single value");
      }
      u = *(uint32_t *)((char *)msg->payload() + sizeof(config_descriptor_t));

      msg->unref();
      msg = NULL;

    } catch (Exception &e) {
      e.append("NetworkConfiguration::get_uint: Fetching unsigned int failed");
      if ( msg != NULL ) {
	msg->unref();
	msg = NULL;
      }
      mutex->unlock();
      throw;
    }
  }

  mutex->unlock();

  return u;
}


int
NetworkConfiguration::get_int(const char *path)
{
  if ( strlen(path) > CONFIG_MSG_PATH_LENGTH ) {
    throw OutOfBoundsException("NetworkConfiguration::get_int: "
			       "Maximum length for path exceeded");
  }
  if ( ! __connected ) {
    throw ConnectionDiedException("NetworkConfiguration: Cannot send get, "
				  "client connection is not alive");
  }

  int i;
  mutex->lock();

  if ( __mirror_mode ) {
    try {
      i = mirror_config->get_int(path);
    } catch (Exception &e) {
      e.append("NetworkConfiguration[mirroring]::get_int: exception in mirror database");
      mutex->unlock();
      throw;
    }
  } else {
    try {
      send_get(path, MSG_CONFIG_GET_INT);

      config_descriptor_t *d = msg->msgge<config_descriptor_t>();
      if (d->num_values > 0) {
	msg->unref();
	msg = NULL;
	throw Exception("NetworkConfiguration: received list of and not a single value");
      }
      i = *(int32_t *)((char *)msg->payload() + sizeof(config_descriptor_t));

      msg->unref();
      msg = NULL;

    } catch (Exception &e) {
      e.append("NetworkConfiguration::get_int: Fetching int failed");
      if ( msg != NULL ) {
	msg->unref();
	msg = NULL;
      }
      mutex->unlock();
      throw;
    }
  }

  mutex->unlock();

  return i;
}


bool
NetworkConfiguration::get_bool(const char *path)
{
  if ( strlen(path) > CONFIG_MSG_PATH_LENGTH ) {
    throw OutOfBoundsException("NetworkConfiguration::get_bool: "
			       "Maximum length for path exceeded");
  }
  if ( ! __connected ) {
    throw ConnectionDiedException("NetworkConfiguration: Cannot send get, "
				  "client connection is not alive");
  }

  bool b;
  mutex->lock();

  if ( __mirror_mode ) {
    try {
      b = mirror_config->get_bool(path);
    } catch (Exception &e) {
      e.append("NetworkConfiguration[mirroring]::get_bool: exception in mirror database");
      mutex->unlock();
      throw;
    }
  } else {
    try {
      send_get(path, MSG_CONFIG_GET_BOOL);

      config_descriptor_t *d = msg->msgge<config_descriptor_t>();
      if (d->num_values > 0) {
	msg->unref();
	msg = NULL;
	throw Exception("NetworkConfiguration: received list of and not a single value");
      }
      b = (*(int32_t *)((char *)msg->payload() + sizeof(config_descriptor_t)) != 0);

      msg->unref();
      msg = NULL;

    } catch (Exception &e) {
      e.append("NetworkConfiguration::get_bool: Fetching bool failed");
      if ( msg != NULL ) {
	msg->unref();
	msg = NULL;
      }
      mutex->unlock();
      throw;
    }
  }

  mutex->unlock();

  return b;
}


std::string
NetworkConfiguration::get_string(const char *path)
{
  if ( strlen(path) > CONFIG_MSG_PATH_LENGTH ) {
    throw OutOfBoundsException("NetworkConfiguration::get_string: "
			       "Maximum length for path exceeded");
  }
  if ( ! __connected ) {
    throw ConnectionDiedException("NetworkConfiguration: Cannot send get, "
				  "client connection is not alive");
  }

  std::string s;
  mutex->lock();

  if ( __mirror_mode ) {
    try {
      s = mirror_config->get_string(path);
    } catch (Exception &e) {
      e.append("NetworkConfiguration[mirroring]::get_string: exception in mirror database");
      mutex->unlock();
      throw;
    }
  } else {
    try {
      send_get(path, MSG_CONFIG_GET_STRING);

      config_descriptor_t *d = msg->msgge<config_descriptor_t>();
      if (d->num_values > 0) {
	msg->unref();
	msg = NULL;
	throw Exception("NetworkConfiguration: received list of and not a single value");
      }
      config_string_value_t *sv =
	(config_string_value_t *)((char *)msg->payload() + sizeof(config_descriptor_t));
      s = (char *)sv + sizeof(config_string_value_t);

      msg->unref();
      msg = NULL;

    } catch (Exception &e) {
      e.append("NetworkConfiguration::get_string: Fetching int failed");
      if ( msg != NULL ) {
	msg->unref();
	msg = NULL;
      }
      mutex->unlock();
      throw;
    }
  }

  mutex->unlock();

  return s;
}

std::vector<float>
NetworkConfiguration::get_floats(const char *path)
{
  throw NotImplementedException("NetworkConf: list values are not supported");
}

std::vector<unsigned int>
NetworkConfiguration::get_uints(const char *path)
{
  throw NotImplementedException("NetworkConf: list values are not supported");
}

std::vector<int>
NetworkConfiguration::get_ints(const char *path)
{
  throw NotImplementedException("NetworkConf: list values are not supported");
}

std::vector<bool>
NetworkConfiguration::get_bools(const char *path)
{
  throw NotImplementedException("NetworkConf: list values are not supported");
}

std::vector<std::string>
NetworkConfiguration::get_strings(const char *path)
{
  throw NotImplementedException("NetworkConf: list values are not supported");
}


std::string
NetworkConfiguration::get_comment(const char *path)
{
  if ( strlen(path) > CONFIG_MSG_PATH_LENGTH ) {
    throw OutOfBoundsException("NetworkConfiguration::get_comment: "
			       "Maximum length for path exceeded");
  }
  if ( ! __connected ) {
    throw ConnectionDiedException("NetworkConfiguration: Cannot send get, "
				  "client connection is not alive");
  }

  std::string s;
  mutex->lock();

  if ( __mirror_mode ) {
    try {
      s = mirror_config->get_comment(path);
    } catch (Exception &e) {
      e.append("NetworkConfiguration[mirroring]::get_comment: exception in mirror database");
      mutex->unlock();
      throw;
    }
  } else {
    try {
      send_get(path, MSG_CONFIG_GET_COMMENT);

      config_comment_msg_t *sm = msg->msgge<config_comment_msg_t>();
      s = sm->s;

      msg->unref();
      msg = NULL;

    } catch (Exception &e) {
      e.append("NetworkConfiguration::get_comment: Fetching int failed");
      if ( msg != NULL ) {
	msg->unref();
	msg = NULL;
      }
      mutex->unlock();
      throw;
    }
  }

  mutex->unlock();

  return s;
}


std::string
NetworkConfiguration::get_default_comment(const char *path)
{
  if ( strlen(path) > CONFIG_MSG_PATH_LENGTH ) {
    throw OutOfBoundsException("NetworkConfiguration::get_default_comment: "
			       "Maximum length for path exceeded");
  }
  if ( ! __connected ) {
    throw ConnectionDiedException("NetworkConfiguration: Cannot send get, "
				  "client connection is not alive");
  }

  std::string s;
  mutex->lock();

  if ( __mirror_mode ) {
    try {
      s = mirror_config->get_default_comment(path);
    } catch (Exception &e) {
      e.append("NetworkConfiguration[mirroring]::get_default_comment: "
	       "exception in mirror database");
      mutex->unlock();
      throw;
    }
  } else {
    try {
      send_get(path, MSG_CONFIG_GET_DEFAULT_COMMENT);

      config_comment_msg_t *sm = msg->msgge<config_comment_msg_t>();
      s = sm->s;

      msg->unref();
      msg = NULL;

    } catch (Exception &e) {
      e.append("NetworkConfiguration::get_comment: Fetching int failed");
      if ( msg != NULL ) {
	msg->unref();
	msg = NULL;
      }
      mutex->unlock();
      throw;
    }
  }

  mutex->unlock();

  return s;
}


Configuration::ValueIterator *
NetworkConfiguration::get_value(const char *path)
{
  if ( strlen(path) > CONFIG_MSG_PATH_LENGTH ) {
    throw OutOfBoundsException("NetworkConfiguration::get_value: "
			       "Maximum length for path exceeded");
  }
  if ( ! __connected ) {
    throw ConnectionDiedException("NetworkConfiguration: Cannot send get, "
				  "client connection is not alive");
  }

  Configuration::ValueIterator *i;
  mutex->lock();

  if ( __mirror_mode ) {
    try {
      i = mirror_config->get_value(path);
    } catch (Exception &e) {
      e.append("NetworkConfiguration[mirroring]::get_float: exception in mirror database");
      mutex->unlock();
      throw;
    }
  } else {
    config_getval_msg_t *g = (config_getval_msg_t *)calloc(1, sizeof(config_getval_msg_t));
    strncpy(g->cp.path, path, CONFIG_MSG_PATH_LENGTH);
    FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_CONFIGMANAGER,
							  MSG_CONFIG_GET_VALUE,
							  g, sizeof(config_getval_msg_t));
    c->enqueue_and_wait(omsg);

    if ( msg == NULL ) {
      mutex->unlock();
      throw NullPointerException("NetworkConfiguration::get_value: msg == NULL");
    }

    i = new NetConfValueIterator(msg);

    msg->unref();
    msg = NULL;
  }

  mutex->unlock();

  return i;
}


void
NetworkConfiguration::set_value_internal(unsigned int msg_type,
					 const char *path, uint16_t num_values,
					 size_t data_size, void *data)
{
  if ( strlen(path) > CONFIG_MSG_PATH_LENGTH ) {
    throw OutOfBoundsException("NetworkConfiguration::set_float: "
			       "Maximum length for path exceeded");
  }
  if ( ! __connected ) {
    throw ConnectionDiedException("NetworkConfiguration: Cannot set value, "
				  "client connection is not alive");
  }

  mutex->lock();
  size_t msg_size = sizeof(config_descriptor_t) + data_size;
  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_CONFIGMANAGER,
							msg_type, msg_size);
  config_descriptor_t *cd = omsg->msgge<config_descriptor_t>();
  strncpy(cd->path, path, CONFIG_MSG_PATH_LENGTH);
  cd->num_values = num_values;

  void *mdata = ((char *)msg->payload() + sizeof(config_descriptor_t));
  memcpy(mdata, data, data_size);

  c->enqueue_and_wait(omsg);
  if ( ! __mirror_mode && (msg != NULL) ) {
    msg->unref();
    msg = NULL;
  }
  mutex->unlock();
}

void
NetworkConfiguration::set_float(const char *path, float f)
{
  set_value_internal(MSG_CONFIG_SET_FLOAT, path, 0, sizeof(float), &f);
}


void
NetworkConfiguration::set_default_float(const char *path, float f)
{
  set_value_internal(MSG_CONFIG_SET_DEFAULT_FLOAT, path, 0, sizeof(float), &f);
}


void
NetworkConfiguration::set_uint(const char *path, unsigned int uint)
{
  set_value_internal(MSG_CONFIG_SET_UINT, path, 0, sizeof(uint32_t), &uint);
}


void
NetworkConfiguration::set_default_uint(const char *path, unsigned int uint)
{
  set_value_internal(MSG_CONFIG_SET_DEFAULT_UINT, path, 0, sizeof(uint32_t), &uint);
}


void
NetworkConfiguration::set_int(const char *path, int i)
{
  set_value_internal(MSG_CONFIG_SET_INT, path, 0, sizeof(int32_t), &i);
}


void
NetworkConfiguration::set_default_int(const char *path, int i)
{
  set_value_internal(MSG_CONFIG_SET_DEFAULT_INT, path, 0, sizeof(int32_t), &i);
}


void
NetworkConfiguration::set_bool(const char *path, bool b)
{
  int32_t bs[1] = { b ? 1 : 0 };
  set_value_internal(MSG_CONFIG_SET_BOOL, path, 0, sizeof(int32_t), bs);
}


void
NetworkConfiguration::set_default_bool(const char *path, bool b)
{
  int32_t bs[1] = { b ? 1 : 0 };
  set_value_internal(MSG_CONFIG_SET_DEFAULT_BOOL, path, 0, sizeof(int32_t), bs);
}


void
NetworkConfiguration::set_string(const char *path, const char *s)
{
  size_t s_length = strlen(s);
  size_t data_size = sizeof(config_string_value_t) + s_length;
  void *data = malloc(data_size);
  config_string_value_t *sv = (config_string_value_t *)data;
  sv->s_length = s_length;
  strncpy((char *)sv + sizeof(config_string_value_t), s, s_length);
  set_value_internal(MSG_CONFIG_SET_STRING, path, 0, data_size, data);
  free(data);
}


void
NetworkConfiguration::set_default_string(const char *path, const char *s)
{
  size_t s_length = strlen(s);
  size_t data_size = sizeof(config_string_value_t) + s_length;
  void *data = malloc(data_size);
  config_string_value_t *sv = (config_string_value_t *)data;
  sv->s_length = s_length;
  strncpy((char *)sv + sizeof(config_string_value_t), s, s_length);
  set_value_internal(MSG_CONFIG_SET_DEFAULT_STRING, path, 0, data_size, data);
  free(data);
}


void
NetworkConfiguration::set_string(const char *path, std::string &s)
{
  set_string(path, s.c_str());
}


void
NetworkConfiguration::set_default_string(const char *path, std::string &s)
{
  set_default_string(path, s.c_str());
}


void
NetworkConfiguration::set_floats(const char *path, std::vector<float> &f)
{
  throw NotImplementedException("NetworkConf: list values are not supported");
}

void
NetworkConfiguration::set_uints(const char *path, std::vector<unsigned int> &u)
{
  throw NotImplementedException("NetworkConf: list values are not supported");
}

void
NetworkConfiguration::set_ints(const char *path, std::vector<int> &i)
{
  throw NotImplementedException("NetworkConf: list values are not supported");
}

void
NetworkConfiguration::set_bools(const char *path, std::vector<bool> &b)
{
  throw NotImplementedException("NetworkConf: list values are not supported");
}

void
NetworkConfiguration::set_strings(const char *path, std::vector<std::string> &s)
{
  throw NotImplementedException("NetworkConf: list values are not supported");
}

void
NetworkConfiguration::set_strings(const char *path, std::vector<const char *> &s)
{
  throw NotImplementedException("NetworkConf: list values are not supported");
}



void
NetworkConfiguration::set_comment(const char *path, const char *comment)
{
}


void
NetworkConfiguration::set_default_comment(const char *path, const char *comment)
{
}


void
NetworkConfiguration::set_comment(const char *path, std::string &comment)
{
}


void
NetworkConfiguration::set_default_comment(const char *path, std::string &comment)
{
}


void
NetworkConfiguration::erase_internal(const char *path, bool is_default)
{
  if ( strlen(path) > CONFIG_MSG_PATH_LENGTH ) {
    throw OutOfBoundsException("NetworkConfiguration::erase: "
			       "Maximum length for path exceeded");
  }
  if ( ! __connected ) {
    throw ConnectionDiedException("NetworkConfiguration: Cannot set value, "
				  "client connection is not alive");
  }

  mutex->lock();
  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_CONFIGMANAGER,
							MSG_CONFIG_ERASE_VALUE,
							sizeof(config_erase_value_msg_t));
  config_erase_value_msg_t *m = omsg->msg<config_erase_value_msg_t>();
  m->cp.is_default = is_default ? 1 : 0;
  strncpy(m->cp.path, path, CONFIG_MSG_PATH_LENGTH);
  c->enqueue_and_wait(omsg);
  if ( ! __mirror_mode && (msg != NULL) ) {
    msg->unref();
    msg = NULL;
  }
  mutex->unlock();
}


void
NetworkConfiguration::erase(const char *path)
{
  erase_internal(path, /*default */ false);
}


void
NetworkConfiguration::erase_default(const char *path)
{
  erase_internal(path, /*default */ true);
}


/** We are no longer registered in Fawkes network client.
 * Ignored.
 * @param id the id of the calling client
 */
void
NetworkConfiguration::deregistered(unsigned int id) throw()
{
}


void
NetworkConfiguration::inbound_received(FawkesNetworkMessage *m,
				       unsigned int id) throw()
{
  if ( m->cid() == FAWKES_CID_CONFIGMANAGER ) {

    if ( __mirror_mode ) {
      switch (m->msgid()) {
      case MSG_CONFIG_LIST:
	// put all values into mirror database
	{
	  //mirror_config->transaction_begin();
	  ConfigListContent *clc = m->msgc<ConfigListContent>();
	  while ( clc->has_next() ) {
	    size_t cle_size = 0;
	    config_list_entity_header_t *cle = clc->next(&cle_size);
	    switch ( cle->type ) {
	    case MSG_CONFIG_FLOAT_VALUE:
	      {
		float *msg_values =
		  (float *)((char *)cle + sizeof(config_list_entity_header_t));
		if (cle->cp.num_values > 1) {
		  std::vector<float> values(cle->cp.num_values, 0);
		  for (unsigned int j = 0; j < cle->cp.num_values; ++j) {
		    values[j] = msg_values[j];
		  }
		  mirror_config->set_floats(cle->cp.path, values);
		} else {
		  if ( cle->cp.is_default ) {
		    mirror_config->set_default_float(cle->cp.path, *msg_values);
		  } else {
		    mirror_config->set_float(cle->cp.path, *msg_values);
		  }
		}
	      }
	      break;

	    case MSG_CONFIG_INT_VALUE:
	      {
		int32_t *msg_values =
		  (int32_t *)((char *)cle + sizeof(config_list_entity_header_t));
		if (cle->cp.num_values > 1) {
		  std::vector<int32_t> values(cle->cp.num_values, 0);
		  for (unsigned int j = 0; j < cle->cp.num_values; ++j) {
		    values[j] = msg_values[j];
		  }
		  mirror_config->set_ints(cle->cp.path, values);
		} else {
		  if ( cle->cp.is_default ) {
		    mirror_config->set_default_int(cle->cp.path, *msg_values);
		  } else {
		    mirror_config->set_int(cle->cp.path, *msg_values);
		  }
		}
	      }
	      break;

	    case MSG_CONFIG_UINT_VALUE:
	      {
		uint32_t *msg_values =
		  (uint32_t *)((char *)cle + sizeof(config_list_entity_header_t));
		if (cle->cp.num_values > 1) {
		  std::vector<uint32_t> values(cle->cp.num_values, 0);
		  for (unsigned int j = 0; j < cle->cp.num_values; ++j) {
		    values[j] = msg_values[j];
		  }
		  mirror_config->set_uints(cle->cp.path, values);
		} else {
		  if ( cle->cp.is_default ) {
		    mirror_config->set_default_uint(cle->cp.path, *msg_values);
		  } else {
		    mirror_config->set_uint(cle->cp.path, *msg_values);
		  }
		}
	      }
	      break;

	    case MSG_CONFIG_BOOL_VALUE:
	      {
		int32_t *msg_values =
		  (int32_t *)((char *)cle + sizeof(config_list_entity_header_t));
		if (cle->cp.num_values > 1) {
		  std::vector<bool> values(cle->cp.num_values, 0);
		  for (unsigned int j = 0; j < cle->cp.num_values; ++j) {
		    values[j] = (msg_values[j] != 0);
		  }
		  mirror_config->set_bools(cle->cp.path, values);
		} else {
		  if ( cle->cp.is_default ) {
		    mirror_config->set_default_bool(cle->cp.path, (*msg_values != 0));
		  } else {
		    mirror_config->set_bool(cle->cp.path, (*msg_values != 0));
		  }
		}
	      }
	      break;

	    case MSG_CONFIG_STRING_VALUE:
	      {
		char *tmpdata = (char *)cle + sizeof(config_list_entity_header_t);
		if (cle->cp.num_values > 1) {
		  std::vector<std::string> values(cle->cp.num_values, "");
		  for (unsigned int j = 0; j < cle->cp.num_values; ++j) {
		    config_string_value_t *csv = (config_string_value_t *)tmpdata;
		    char *msg_string = tmpdata + sizeof(config_string_value_t);
		    values[j] = std::string(msg_string, csv->s_length);
		    tmpdata += sizeof(config_string_value_t) + csv->s_length + 1;
		  }
		  mirror_config->set_strings(cle->cp.path, values);
		} else {
		  config_string_value_t *csv = (config_string_value_t *)tmpdata;
		  char *msg_string = tmpdata + sizeof(config_string_value_t);
		  if ( cle->cp.is_default ) {
		    mirror_config->set_default_string(cle->cp.path,
						      std::string(msg_string, csv->s_length).c_str());
		  } else {
		    mirror_config->set_string(cle->cp.path,
					      std::string(msg_string, csv->s_length).c_str());
		  }
		}
	      }
	      break;

	    case MSG_CONFIG_COMMENT_VALUE:
	      // ignored
	      break;
	    }
	  }
	  //mirror_config->transaction_commit();
	  delete clc;
	}

	// add all change handlers
	for (ChangeHandlerMultimap::const_iterator j = _change_handlers.begin(); j != _change_handlers.end(); ++j) {
	  _ch_range = _change_handlers.equal_range((*j).first);
	  for (ChangeHandlerMultimap::const_iterator i = _ch_range.first; i != _ch_range.second; ++i) {
	    mirror_config->add_change_handler((*i).second);
	  }
	}
	// initial answer received -> wake up set_mirror_mode()
	if (__mirror_init_barrier) __mirror_init_barrier->wait();
	break;

      case MSG_CONFIG_VALUE_ERASED:
	try {
	  config_value_erased_msg_t *em = m->msg<config_value_erased_msg_t>();
	  if (em->cp.is_default == 1) {
	    mirror_config->erase_default(em->cp.path);
	  } else {
	    mirror_config->erase(em->cp.path);
	  }
	} catch (Exception &e) {
	  // Just ignore silently
	  LibLogger::log_warn("NetworkConfiguration", "[mirroring]::inboundReceived: erasing failed");
	}
	break;

      case MSG_CONFIG_FLOAT_VALUE:
	try {
	  config_descriptor_t *cd = m->msgge<config_descriptor_t>();
	  if (cd->num_values > 1) {
	    float *fs = (float *)((char *)msg->payload() + sizeof(config_descriptor_t));
	    std::vector<float> floats(cd->num_values, 0.0);
	    for (unsigned int i = 0; i < cd->num_values; ++i) {
	      floats[i] = fs[i];
	    }
	    mirror_config->set_floats(cd->path, floats);
	  } else {
	    float f = *(float *)((char *)msg->payload() + sizeof(config_descriptor_t));
	    if (cd->is_default == 1) {
	      mirror_config->set_default_float(cd->path, f);
	    } else {
	      mirror_config->set_float(cd->path, f);
	    }
	  }
	} catch (TypeMismatchException &e) {
	  // Just ignore silently
	  LibLogger::log_warn("NetworkConfiguration", "[mirroring]::inboundReceived: invalid float received");
	}
	break;

      case MSG_CONFIG_UINT_VALUE:
	try {
	  config_descriptor_t *cd = m->msgge<config_descriptor_t>();
	  if (cd->num_values > 1) {
	    uint32_t *vs = (uint32_t *)((char *)msg->payload() + sizeof(config_descriptor_t));
	    std::vector<unsigned int> values(cd->num_values, 0);
	    for (unsigned int i = 0; i < cd->num_values; ++i) {
	      values[i] = vs[i];
	    }
	    mirror_config->set_uints(cd->path, values);
	  } else {
	    unsigned int u =
	      *(uint32_t *)((char *)msg->payload() + sizeof(config_descriptor_t));

	    if (cd->is_default == 1) {
	      mirror_config->set_default_uint(cd->path, u);
	    } else {
	      mirror_config->set_uint(cd->path, u);
	    }
	  }
	} catch (TypeMismatchException &e) {
	  // Just ignore silently
	  LibLogger::log_warn("NetworkConfiguration", "[mirroring]::inboundReceived: invalid uint received");
	}
	break;

      case MSG_CONFIG_INT_VALUE:
	try {
	  config_descriptor_t *cd = m->msgge<config_descriptor_t>();
	  if (cd->num_values > 1) {
	    int32_t *vs = (int32_t *)((char *)msg->payload() + sizeof(config_descriptor_t));
	    std::vector<int> values(cd->num_values, 0);
	    for (unsigned int i = 0; i < cd->num_values; ++i) {
	      values[i] = vs[i];
	    }
	    mirror_config->set_ints(cd->path, values);
	  } else {
	    unsigned int i =
	      *(int32_t *)((char *)msg->payload() + sizeof(config_descriptor_t));

	    if (cd->is_default == 1) {
	      mirror_config->set_default_int(cd->path, i);
	    } else {
	      mirror_config->set_int(cd->path, i);
	    }
	  }
	} catch (TypeMismatchException &e) {
	  // Just ignore silently
	  LibLogger::log_warn("NetworkConfiguration", "[mirroring]::inboundReceived: invalid int received");
	}
	break;

      case MSG_CONFIG_BOOL_VALUE:
	try {
	  config_descriptor_t *cd = m->msgge<config_descriptor_t>();
	  if (cd->num_values > 1) {
	    int32_t *vs = (int32_t *)((char *)msg->payload() + sizeof(config_descriptor_t));
	    std::vector<bool> values(cd->num_values, 0);
	    for (unsigned int i = 0; i < cd->num_values; ++i) {
	      values[i] = (vs[i] != 0);
	    }
	    mirror_config->set_bools(cd->path, values);
	  } else {
	    unsigned int i =
	      *(int32_t *)((char *)msg->payload() + sizeof(config_descriptor_t));

	    if (cd->is_default == 1) {
	      mirror_config->set_default_bool(cd->path, (i != 0));
	    } else {
	      mirror_config->set_bool(cd->path, (i != 0));
	    }
	  }
	} catch (TypeMismatchException &e) {
	  // Just ignore silently
	  LibLogger::log_warn("NetworkConfiguration", "[mirroring]::inboundReceived: invalid bool received");
	}
	break;

      case MSG_CONFIG_STRING_VALUE:
	try {
	  config_descriptor_t *cd = m->msgge<config_descriptor_t>();
	  if (cd->num_values > 1) {
	    std::vector<std::string> values(cd->num_values, "");
	    size_t pos = sizeof(config_descriptor_t);
	    for (unsigned int i = 0; i < cd->num_values; ++i) {
	      config_string_value_t *vs =
		(config_string_value_t *)((char *)msg->payload() + pos);
	      char *msg_string =
		((char *)msg->payload() + pos) + sizeof(config_string_value_t);
	      values[i] = std::string(msg_string, vs->s_length);
	      pos += sizeof(config_string_value_t) + vs->s_length + 1;
	    }
	    mirror_config->set_strings(cd->path, values);
	  } else {
	    config_string_value_t *sv =
	      (config_string_value_t *)((char *)msg->payload() + sizeof(config_descriptor_t));
	    char *msg_string =
	      (char *)msg->payload() + sizeof(config_string_value_t);

	    std::string value = std::string(msg_string, sv->s_length);
	    if (cd->is_default == 1) {
	      mirror_config->set_default_string(cd->path, value);
	    } else {
	      mirror_config->set_string(cd->path, value);
	    }
	  }
	} catch (TypeMismatchException &e) {
	  // Just ignore silently
	  LibLogger::log_warn("NetworkConfiguration", "[mirroring]::inboundReceived: invalid string received");
	}
	break;

      case MSG_CONFIG_COMMENT_VALUE:
	try {
	  config_comment_msg_t *cm = m->msgge<config_comment_msg_t>();
	  if (cm->cp.is_default == 1) {
	    mirror_config->set_default_comment(cm->cp.path, cm->s);
	  } else {
	    mirror_config->set_comment(cm->cp.path, cm->s);
	  }
	} catch (TypeMismatchException &e) {
	  // Just ignore silently
	  LibLogger::log_warn("NetworkConfiguration", "[mirroring]::inboundReceived: invalid string received");
	}
	break;
      }
    } else {
      msg = m;
      msg->ref();
    }
  }
}


void
NetworkConfiguration::connection_died(unsigned int id) throw()
{
  __connected = false;
  __mirror_mode_before_connection_dead = __mirror_mode;
  set_mirror_mode(false);
  mutex->unlock(); //Just in case...
}


void
NetworkConfiguration::connection_established(unsigned int id) throw()
{
  __connected = true;
  set_mirror_mode(__mirror_mode_before_connection_dead);
}


void
NetworkConfiguration::add_change_handler(ConfigurationChangeHandler *h)
{
  Configuration::add_change_handler(h);

  if ( __mirror_mode ) {
    mirror_config->add_change_handler(h);
  }
}


void
NetworkConfiguration::rem_change_handler(ConfigurationChangeHandler *h)
{
  Configuration::rem_change_handler(h);
  if ( __mirror_mode ) {
    mirror_config->rem_change_handler(h);
  }
}


/** Enable or disable mirror mode.
 * @param mirror true to enable mirror mode, false to disable
 */
void
NetworkConfiguration::set_mirror_mode(bool mirror)
{
  if ( mirror ) {
    if ( ! __mirror_mode ) {

      if ( ! __connected ) {
	throw CannotEnableMirroringException("Client connection is dead");
      }

      mirror_config = new MemoryConfiguration();

      __mirror_init_barrier = new InterruptibleBarrier(2);
      mutex->lock();

      __mirror_mode = true;

      // subscribe
      FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_CONFIGMANAGER,
							    MSG_CONFIG_SUBSCRIBE);
      c->enqueue(omsg);

      // wait until all data has been received (or timeout)
      if (! __mirror_init_barrier->wait(__mirror_timeout_sec, 0)) {
        // timeout
        delete mirror_config;
	__mirror_init_barrier = NULL;
	delete __mirror_init_barrier;
        mutex->unlock();
        throw CannotEnableMirroringException("Didn't receive data in time");
      }
      mutex->unlock();
      delete __mirror_init_barrier;
      __mirror_init_barrier = NULL;
    }
  } else {
    if ( __mirror_mode ) {
      __mirror_mode = false;
      // unsubscribe
      if ( __connected ) {
	FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_CONFIGMANAGER,
							      MSG_CONFIG_UNSUBSCRIBE);
	c->enqueue(omsg);
      }

      // delete local temporary mirror database
      delete mirror_config;
    }
  }
}


void
NetworkConfiguration::lock()
{
  mutex->lock();
}


bool
NetworkConfiguration::try_lock()
{
  return mutex->try_lock();
}


void
NetworkConfiguration::unlock()
{
  mutex->unlock();
}


void
NetworkConfiguration::try_dump()
{
}


Configuration::ValueIterator *
NetworkConfiguration::iterator()
{
  if ( __mirror_mode ) {
    return mirror_config->iterator();
  } else {
    throw Exception("NetworkConfiguration: Iterating only supported in mirror mode");
  }
}


/** Iterator for all default values.
 * Returns an iterator that can be used to iterate over all default values in
 * the current default configuration. Note that this might return less paths than
 * available, because the values for which no default entry exists are not
 * returned.
 * @return iterator over all default values
 */
Configuration::ValueIterator *
NetworkConfiguration::iterator_default()
{
  if ( __mirror_mode ) {
    return mirror_config->iterator_default();
  } else {
    throw Exception("NetworkConfiguration: Iterating only supported in mirror mode");
  }
}


/** Iterator for all host-specific values.
 * Returns an iterator that can be used to iterate over all host-specific values
 * in the current configuration. Note that this might return less paths than
 * available, because the default values for which no host-specific entry exists
 * are not returned.
 * @return iterator over all host-specific values
 */
Configuration::ValueIterator *
NetworkConfiguration::iterator_hostspecific()
{
  if ( __mirror_mode ) {
    return mirror_config->iterator_hostspecific();
  } else {
    throw Exception("NetworkConfiguration: Iterating only supported in mirror mode");
  }
}


Configuration::ValueIterator *
NetworkConfiguration::search(const char *path)
{
  if ( __mirror_mode ) {
    return mirror_config->search(path);
  } else {
    throw Exception("NetworkConfiguration: Searching only supported in mirror mode");
  }
}


/** @class NetworkConfiguration::NetConfValueIterator <config/netconf.h>
 * Network configuration value iterator.
 * @author Tim Niemueller
 */


/** Constructor.
 * @param i internal other iterator, for instance form local mirrored database.
 */
NetworkConfiguration::NetConfValueIterator::NetConfValueIterator(Configuration::ValueIterator *i)
{
  // not interesting in this case, but anyway...
  iterated_once = false;
  this->i = i;
  msg = NULL;
  _path = NULL;
}


/** Constructor.
 * Returns invalid iterator.
 */
NetworkConfiguration::NetConfValueIterator::NetConfValueIterator()
{
  // not interesting in this case, but anyway...
  iterated_once = false;
  i = NULL;
  msg = NULL;
  _path = NULL;
}


/** Constructor.
 * Internally holds a message. Only this one value is accessible.
 * @param m message
 */
NetworkConfiguration::NetConfValueIterator::NetConfValueIterator(FawkesNetworkMessage *m)
{
  i = NULL;
  msg = NULL;
  iterated_once = false;
  _path = NULL;

  if ( (m->cid() == FAWKES_CID_CONFIGMANAGER) &&
       (m->msgid() >= MSG_CONFIG_VALUE_BEGIN) &&
       (m->msgid() <= MSG_CONFIG_VALUE_END) &&
       (m->payload_size() > sizeof(config_descriptor_t)) ) {
    msg = m;
    msg->ref();
    // extract path
    // all messages start with config_descriptor!
    _path      = (char *)malloc(CONFIG_MSG_PATH_LENGTH + 1);
    _path[CONFIG_MSG_PATH_LENGTH] = 0;
    config_descriptor_t *cd = (config_descriptor_t *)msg->payload();
    strncpy(_path, cd->path, CONFIG_MSG_PATH_LENGTH);
  } else {
    // invalid value, maybe path does not exist!
  }
}


/** Destructor. */
NetworkConfiguration::NetConfValueIterator::~NetConfValueIterator()
{
  delete i;
  if ( msg != NULL )         msg->unref();
  if ( _path != NULL)        free(_path);
}


bool
NetworkConfiguration::NetConfValueIterator::next()
{
  if ( i == NULL) {
    if ( (msg == NULL) || iterated_once ) {
      return false;
    } else {
      iterated_once = true;
      return true;
    }
  } else {
    return i->next();
  }
}


bool
NetworkConfiguration::NetConfValueIterator::valid() const
{
  return ( (i != NULL) || (msg != NULL) );
}


const char *
NetworkConfiguration::NetConfValueIterator::path() const
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access path on invalid iterator");
    } else {
      return _path;
    }
  } else {
    return i->path();
  }
}


const char *
NetworkConfiguration::NetConfValueIterator::type() const
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access path on invalid iterator");
    }

    switch (msg->msgid()) {
    case MSG_CONFIG_FLOAT_VALUE:   return "float";
    case MSG_CONFIG_UINT_VALUE:    return "unsigned int";
    case MSG_CONFIG_INT_VALUE:     return "int";
    case MSG_CONFIG_BOOL_VALUE:    return "bool";
    case MSG_CONFIG_STRING_VALUE:  return "string";
    default:
      throw NullPointerException("Unknown type in NetConfValueIterator");
    }
  } else {
    return i->type();
  }
}


bool
NetworkConfiguration::NetConfValueIterator::is_float() const
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on invalid iterator");
    }
    return (msg->msgid() == MSG_CONFIG_FLOAT_VALUE);
  } else {
    return i->is_float();
  }
}


bool
NetworkConfiguration::NetConfValueIterator::is_uint() const
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on invalid iterator");
    }
    return (msg->msgid() == MSG_CONFIG_UINT_VALUE);
  } else {
    return i->is_float();
  }
}


bool
NetworkConfiguration::NetConfValueIterator::is_int() const
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on invalid iterator");
    }
    return (msg->msgid() == MSG_CONFIG_INT_VALUE);
  } else {
    return i->is_int();
  }
}


bool
NetworkConfiguration::NetConfValueIterator::is_bool() const
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on invalid iterator");
    }
    return (msg->msgid() == MSG_CONFIG_BOOL_VALUE);
  } else {
    return i->is_bool();
  }
}


bool
NetworkConfiguration::NetConfValueIterator::is_string() const
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on invalid iterator");
    }
    return (msg->msgid() == MSG_CONFIG_STRING_VALUE);
  } else {
    return i->is_string();
  }
}


bool
NetworkConfiguration::NetConfValueIterator::is_list() const
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on invalid iterator");
    }
    config_descriptor_t *cd = msg->msgge<config_descriptor_t>();
    return cd->num_values > 0;
  } else {
    return i->is_list();
  }
}


size_t
NetworkConfiguration::NetConfValueIterator::get_list_size() const
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on invalid iterator");
    }
    config_descriptor_t *cd = msg->msgge<config_descriptor_t>();
    return cd->num_values;
  } else {
    return i->get_list_size();
  }
}


bool
NetworkConfiguration::NetConfValueIterator::is_default() const
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on invalid iterator");
    } else {
      unsigned int msgid = msg->msgid();
      switch (msgid) {
      case MSG_CONFIG_FLOAT_VALUE:
      case MSG_CONFIG_UINT_VALUE:
      case MSG_CONFIG_INT_VALUE:
      case MSG_CONFIG_BOOL_VALUE:
      case MSG_CONFIG_STRING_VALUE:
	{
	  config_descriptor_t *cd = msg->msgge<config_descriptor_t>();
	  return cd->is_default;
	}
      }

      throw TypeMismatchException("NetworkConfiguration: Neither in mirror mode nor "
				  "iterator to value message");
    }
  } else {
    return i->is_default();
  }
}


float
NetworkConfiguration::NetConfValueIterator::get_float() const
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on invalid iterator");
    }
    if (msg->msgid() == MSG_CONFIG_FLOAT_VALUE) {
      config_descriptor_t *cd = msg->msgge<config_descriptor_t>();
      if (cd->num_values > 1) {
	throw TypeMismatchException("NetConfValueIterator::get_float: list received");
      }
      return *(float *)((char *)msg->payload() + sizeof(config_descriptor_t));
    } else {
      throw TypeMismatchException("NetConfValueIterator::get_float: type mismatch");
    }
  } else {
    return i->get_float();
  }
}


unsigned int
NetworkConfiguration::NetConfValueIterator::get_uint() const
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on invalid iterator");
    }
    if (msg->msgid() == MSG_CONFIG_UINT_VALUE) {
      config_descriptor_t *cd = msg->msgge<config_descriptor_t>();
      if (cd->num_values > 1) {
	throw TypeMismatchException("NetConfValueIterator::get_uint: list received");
      }
      return *(uint32_t *)((char *)msg->payload() + sizeof(config_descriptor_t));
    } else {
      throw TypeMismatchException("NetConfValueIterator::get_uint: type mismatch");
    }
  } else {
    return i->get_int();
  }
}


int
NetworkConfiguration::NetConfValueIterator::get_int() const
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on invalid iterator");
    }
    if (msg->msgid() == MSG_CONFIG_INT_VALUE) {
      config_descriptor_t *cd = msg->msgge<config_descriptor_t>();
      if (cd->num_values > 1) {
	throw TypeMismatchException("NetConfValueIterator::get_int: list received");
      }
      return *(int32_t *)((char *)msg->payload() + sizeof(config_descriptor_t));
    } else {
      throw TypeMismatchException("NetConfValueIterator::get_int: type mismatch");
    }
  } else {
    return i->get_int();
  }
}


bool
NetworkConfiguration::NetConfValueIterator::get_bool() const
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on invalid iterator");
    }
    if (msg->msgid() == MSG_CONFIG_BOOL_VALUE) {
      config_descriptor_t *cd = msg->msgge<config_descriptor_t>();
      if (cd->num_values > 1) {
	throw TypeMismatchException("NetConfValueIterator::get_int: list received");
      }
      return (*(int32_t *)((char *)msg->payload() + sizeof(config_descriptor_t)) != 0);
    } else {
      throw TypeMismatchException("NetConfValueIterator::get_bool: type mismatch");
    }
  } else {
    return i->get_bool();
  }
}


std::string
NetworkConfiguration::NetConfValueIterator::get_string() const
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on invalid iterator");
    }
    if (msg->msgid() == MSG_CONFIG_STRING_VALUE) {
      config_descriptor_t *cd = msg->msgge<config_descriptor_t>();
      if (cd->num_values > 1) {
	throw TypeMismatchException("NetConfValueIterator::get_int: list received");
      }
      config_string_value_t *sv =
	(config_string_value_t *)((char *)msg->payload() + sizeof(config_descriptor_t));
      char *msg_string =
	(char *)msg->payload() + sizeof(config_string_value_t);
      return std::string(msg_string, sv->s_length);
    } else {
      throw TypeMismatchException("NetConfValueIterator::get_string: type mismatch, expected %u, got %u",
				  MSG_CONFIG_STRING_VALUE, msg->msgid());
    }
  } else {
    return i->get_string();
  }
}


std::vector<float>
NetworkConfiguration::NetConfValueIterator::get_floats() const
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on invalid iterator");
    }
    if (msg->msgid() == MSG_CONFIG_FLOAT_VALUE) {
      config_descriptor_t *cd = msg->msgge<config_descriptor_t>();
      if (cd->num_values <= 1) {
	throw TypeMismatchException("NetConfValueIterator::get_floats: not a list");
      }
      float *data = (float *)((char *)cd + sizeof(config_descriptor_t));
      std::vector<float> rv(cd->num_values, 0);

      for (unsigned int j = 0; j < cd->num_values; ++j) {
	rv[j] = data[j];
      }
      return rv;
    } else {
      throw TypeMismatchException("NetConfValueIterator::get_string: type mismatch, expected %u, got %u",
				  MSG_CONFIG_STRING_VALUE, msg->msgid());
    }
  } else {
    return i->get_floats();
  }
}

std::vector<unsigned int>
NetworkConfiguration::NetConfValueIterator::get_uints() const
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on invalid iterator");
    }
    if (msg->msgid() == MSG_CONFIG_UINT_VALUE) {
      config_descriptor_t *cd = msg->msgge<config_descriptor_t>();
      if (cd->num_values <= 1) {
	throw TypeMismatchException("NetConfValueIterator::get_uints: not a list");
      }
      uint32_t *data = (uint32_t *)((char *)cd + sizeof(config_descriptor_t));
      std::vector<unsigned int> rv(cd->num_values, 0);

      for (unsigned int j = 0; j < cd->num_values; ++j) {
	rv[j] = data[j];
      }
      return rv;
    } else {
      throw TypeMismatchException("NetConfValueIterator::get_string: type mismatch, expected %u, got %u",
				  MSG_CONFIG_STRING_VALUE, msg->msgid());
    }
  } else {
    return i->get_uints();
  }
}

std::vector<int>
NetworkConfiguration::NetConfValueIterator::get_ints() const
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on invalid iterator");
    }
    if (msg->msgid() == MSG_CONFIG_INT_VALUE) {
      config_descriptor_t *cd = msg->msgge<config_descriptor_t>();
      if (cd->num_values <= 1) {
	throw TypeMismatchException("NetConfValueIterator::get_ints: not a list");
      }
      int32_t *data = (int32_t *)((char *)cd + sizeof(config_descriptor_t));
      std::vector<int> rv(cd->num_values, 0);

      for (unsigned int j = 0; j < cd->num_values; ++j) {
	rv[j] = data[j];
      }
      return rv;
    } else {
      throw TypeMismatchException("NetConfValueIterator::get_string: type mismatch, expected %u, got %u",
				  MSG_CONFIG_STRING_VALUE, msg->msgid());
    }
  } else {
    return i->get_ints();
  }
}

std::vector<bool>
NetworkConfiguration::NetConfValueIterator::get_bools() const
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on invalid iterator");
    }
    if (msg->msgid() == MSG_CONFIG_INT_VALUE) {
      config_descriptor_t *cd = msg->msgge<config_descriptor_t>();
      if (cd->num_values <= 1) {
	throw TypeMismatchException("NetConfValueIterator::get_ints: not a list");
      }
      int32_t *data = (int32_t *)((char *)cd + sizeof(config_descriptor_t));
      std::vector<bool> rv(cd->num_values, 0);

      for (unsigned int j = 0; j < cd->num_values; ++j) {
	rv[j] = (data[j] != 0);
      }
      return rv;
    } else {
      throw TypeMismatchException("NetConfValueIterator::get_string: type mismatch, expected %u, got %u",
				  MSG_CONFIG_STRING_VALUE, msg->msgid());
    }
  } else {
    return i->get_bools();
  }
}

std::vector<std::string>
NetworkConfiguration::NetConfValueIterator::get_strings() const
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on invalid iterator");
    }
    if (msg->msgid() == MSG_CONFIG_STRING_VALUE) {
      config_descriptor_t *cd = msg->msgge<config_descriptor_t>();
      if (cd->num_values <= 1) {
	throw TypeMismatchException("NetConfValueIterator::get_strings: not a list");
      }
      std::vector<std::string> rv(cd->num_values, "");
      char *tmpdata = (char *)cd + sizeof(config_descriptor_t);

      for (unsigned int j = 0; j < cd->num_values; ++j) {
	config_string_value_t *sv = (config_string_value_t *)tmpdata;
	char *msg_string = tmpdata + sizeof(config_string_value_t);
	rv[j] = std::string(msg_string, sv->s_length);
	tmpdata += sizeof(config_string_value_t) + sv->s_length + 1;
      }
      return rv;
    } else {
      throw TypeMismatchException("NetConfValueIterator::get_string: type mismatch, expected %u, got %u",
				  MSG_CONFIG_STRING_VALUE, msg->msgid());
    }
  } else {
    return i->get_strings();
  }
}


std::string
NetworkConfiguration::NetConfValueIterator::get_as_string() const
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on "
				 "invalid iterator");
    }
    if (msg->msgid() == MSG_CONFIG_STRING_VALUE) {
      return get_string();
    } else if (msg->msgid() == MSG_CONFIG_BOOL_VALUE) {
      return get_bool() ? "true" : "false";
    } else if (msg->msgid() == MSG_CONFIG_INT_VALUE) {
      return StringConversions::to_string(get_int());
    } else if (msg->msgid() == MSG_CONFIG_UINT_VALUE) {
      return StringConversions::to_string(get_uint());
    } else if (msg->msgid() == MSG_CONFIG_FLOAT_VALUE) {
      return StringConversions::to_string(get_float());
    } else {
      throw Exception("NetConfValueIterator::get_as_string: unknown type");
    }
  } else {
    return i->get_as_string();
  }
}


std::string
NetworkConfiguration::NetConfValueIterator::get_comment() const
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on invalid iterator");
    }
    if (msg->msgid() == MSG_CONFIG_COMMENT_VALUE) {
      config_comment_msg_t *cm = msg->msgge<config_comment_msg_t>();
      return cm->s;
    } else {
      throw TypeMismatchException("NetConfValueIterator::get_comment: type mismatch");
    }
  } else {
    return i->get_comment();
  }
}

} // end namespace fawkes
