
/***************************************************************************
 *  netconf.cpp - Fawkes remote configuration access via Fawkes net
 *
 *  Created: Sun Jan 07 15:04:41 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <config/netconf.h>
#include <config/net_messages.h>
#include <config/sqlite.h>

#include <core/threading/mutex.h>
#include <netcomm/fawkes/client.h>
#include <netcomm/fawkes/message.h>

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <stdio.h>

/** @class CannotEnableMirroringException config/netconf.h
 * Thrown if enabling mirror mode failed.
 */


/** Constructor. */
CannotEnableMirroringException::CannotEnableMirroringException()
  : Exception("Could not enable mirroring, could not create tmp files")
{
}


/** @class NetworkConfiguration config/netconf.h
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
 */
NetworkConfiguration::NetworkConfiguration(FawkesNetworkClient *c)
{
  this->c = c;
  try {
    c->registerHandler(this, FAWKES_CID_CONFIGMANAGER);
  } catch (Exception &e) {
    e.append("Failed to register for config manager component on network client");
    throw;
  }
  mutex = new Mutex();
  msg = NULL;
  change_handlers.clear();
  mirror = false;
}


/** Destructor. */
NetworkConfiguration::~NetworkConfiguration()
{
  setMirrorMode(false);
  c->deregisterHandler(FAWKES_CID_CONFIGMANAGER);
  if (msg != NULL) {
    msg->unref();
  }
  delete mutex;
}


/** Load configuration.
 * This is a noop for the NetworkConfiguration.
 * @param name name of the host-based database. This should be a name of the form
 * hostname.db, where hostname is the unqualified part of the hostname.
 * @param defaults_name name of the default database. Should be defaults.db
 * @param tag optional tag to restore
 */
void
NetworkConfiguration::load(const char *name, const char *defaults_name,
			   const char *tag)
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
  mutex->lock();
  copyconf->lock();
  Configuration::ValueIterator *i = copyconf->iterator();
  while ( i->next() ) {
    if ( i->is_float() ) {
      set_float(i->component(), i->path(), i->get_float());
    } else if ( i->is_int() ) {
      set_int(i->component(), i->path(), i->get_int());
    } else if ( i->is_uint() ) {
      set_uint(i->component(), i->path(), i->get_uint());
    } else if ( i->is_bool() ) {
      set_bool(i->component(), i->path(), i->get_bool());
    } else if ( i->is_string() ) {
      set_string(i->component(), i->path(), i->get_string());
    }
  }
  delete i;
  copyconf->unlock();
  mutex->unlock();
}


/** Tag this configuration version.
 * This creates a new tagged version of the current config. The tagged config can be
 * accessed via load().
 * @param tag tag for this version
 */
void
NetworkConfiguration::tag(const char *tag)
{
  mutex->lock();

  mutex->unlock();
}


/** List of tags.
 * @return list of tags
 */
std::list<std::string>
NetworkConfiguration::tags()
{
  mutex->lock();
  std::list<std::string> l;
  mutex->unlock();
  return l;
}


/** Check if a given value exists.
 * @param comp component
 * @param path path to value
 * @return true if the value exists, false otherwise
 */
bool
NetworkConfiguration::exists(const char *comp, const char *path)
{
  return false;
}


/** Get type of value.
 * @param comp component
 * @param path path to value
 * @return string representation of value type
 */
std::string
NetworkConfiguration::get_type(const char *comp, const char *path)
{
  std::string s = "";
  mutex->lock();
  if ( mirror ) {
    s = mirror_config->get_type(comp, path);
  }
  mutex->unlock();
  return s;
}


/** Check if a value is of type float
 * @param comp component
 * @param path path to value
 * @return true if the value exists and is of type float
 */
bool
NetworkConfiguration::is_float(const char *comp, const char *path)
{
  return (get_type(comp, path) == "float");
}


/** Check if a value is of type unsigned int
 * @param comp component
 * @param path path to value
 * @return true if the value exists and is of type unsigned int
 */
bool
NetworkConfiguration::is_uint(const char *comp, const char *path)
{
  return (get_type(comp, path) == "unsigned int");
}


/** Check if a value is of type int
 * @param comp component
 * @param path path to value
 * @return true if the value exists and is of type int
 */
bool
NetworkConfiguration::is_int(const char *comp, const char *path)
{
  return (get_type(comp, path) == "int");
}


/** Check if a value is of type bool
 * @param comp component
 * @param path path to value
 * @return true if the value exists and is of type bool
 */
bool
NetworkConfiguration::is_bool(const char *comp, const char *path)
{
  return (get_type(comp, path) == "bool");
}


/** Check if a value is of type string
 * @param comp component
 * @param path path to value
 * @return true if the value exists and is of type string
 */
bool
NetworkConfiguration::is_string(const char *comp, const char *path)
{
  return (get_type(comp, path) == "string");
}


/** Send get command.
 * @param comp component of value
 * @param path path of value
 * @param msgid msgid for message.
 */
void
NetworkConfiguration::send_get(const char *comp, const char *path, unsigned int msgid)
{
  config_getval_msg_t *g = (config_getval_msg_t *)calloc(1, sizeof(config_getval_msg_t));
  strncpy(g->cp.component, comp, CONFIG_MSG_COMPONENT_LENGTH);
  strncpy(g->cp.path, path, CONFIG_MSG_PATH_LENGTH);
  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_CONFIGMANAGER,
							msgid,
							g, sizeof(config_getval_msg_t));
  c->enqueue(omsg);
  omsg->unref();
  c->wait(FAWKES_CID_CONFIGMANAGER);

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


/** Get value from configuration which is of type float
 * @param comp component
 * @param path path to value
 * @return value
 */
float
NetworkConfiguration::get_float(const char *comp, const char *path)
{
  float f;
  mutex->lock();

  if ( mirror ) {
    try {
      f = mirror_config->get_float(comp, path);
    } catch (Exception &e) {
      e.append("NetworkConfiguration[mirroring]::get_float: exception in mirror database");
      throw;
    }
  } else {
    try {
      send_get(comp, path, MSG_CONFIG_GET_FLOAT);

      config_float_value_msg_t *fm = msg->msg<config_float_value_msg_t>();
      f = fm->f;

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


/** Get value from configuration which is of type unsigned int
 * @param comp component
 * @param path path to value
 * @return value
 */
unsigned int
NetworkConfiguration::get_uint(const char *comp, const char *path)
{
  unsigned int u;
  mutex->lock();

  if ( mirror ) {
    try {
      u = mirror_config->get_uint(comp, path);
    } catch (Exception &e) {
      e.append("NetworkConfiguration[mirroring]::get_uint: exception in mirror database");
      throw;
    }
  } else {
    try {
      send_get(comp, path, MSG_CONFIG_GET_UINT);

      config_uint_value_msg_t *um = msg->msg<config_uint_value_msg_t>();
      u = um->u;

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


/** Get value from configuration which is of type int
 * @param comp component
 * @param path path to value
 * @return value
 */
int
NetworkConfiguration::get_int(const char *comp, const char *path)
{
  int i;
  mutex->lock();

  if ( mirror ) {
    try {
      i = mirror_config->get_int(comp, path);
    } catch (Exception &e) {
      e.append("NetworkConfiguration[mirroring]::get_int: exception in mirror database");
      throw;
    }
  } else {
    try {
      send_get(comp, path, MSG_CONFIG_GET_INT);

      config_int_value_msg_t *im = msg->msg<config_int_value_msg_t>();
      i = im->i;

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


/** Get value from configuration which is of type bool
 * @param comp component
 * @param path path to value
 * @return value
 */
bool
NetworkConfiguration::get_bool(const char *comp, const char *path)
{
  bool b;
  mutex->lock();

  if ( mirror ) {
    try {
      b = mirror_config->get_bool(comp, path);
    } catch (Exception &e) {
      e.append("NetworkConfiguration[mirroring]::get_bool: exception in mirror database");
      throw;
    }
  } else {
    try {
      send_get(comp, path, MSG_CONFIG_GET_BOOL);

      config_bool_value_msg_t *bm = msg->msg<config_bool_value_msg_t>();
      b = (bm->b != 0);

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


/** Get value from configuration which is of type string
 * @param comp component
 * @param path path to value
 * @return value
 */
std::string
NetworkConfiguration::get_string(const char *comp, const char *path)
{
  std::string s;
  mutex->lock();

  if ( mirror ) {
    try {
      s = mirror_config->get_string(comp, path);
    } catch (Exception &e) {
      e.append("NetworkConfiguration[mirroring]::get_string: exception in mirror database");
      throw;
    }
  } else {
    try {
      send_get(comp, path, MSG_CONFIG_GET_STRING);

      config_string_value_msg_t *sm = msg->msg<config_string_value_msg_t>();
      char ts[CONFIG_MSG_MAX_STRING_LENGTH + 1];
      ts[CONFIG_MSG_MAX_STRING_LENGTH] = 0;
      strncpy(ts, sm->s, CONFIG_MSG_MAX_STRING_LENGTH);
      s = ts;

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


/** Get value from configuration.
 * @param comp component
 * @param path path to value
 * @return value iterator for just this one value, maybe invalid if value does not
 * exists.
 */
Configuration::ValueIterator *
NetworkConfiguration::get_value(const char *comp, const char *path)
{
  Configuration::ValueIterator *i;
  mutex->lock();

  if ( mirror ) {
    try {
      i = mirror_config->get_value(comp, path);
    } catch (Exception &e) {
      e.append("NetworkConfiguration[mirroring]::get_float: exception in mirror database");
      throw;
    }
  } else {
    config_getval_msg_t *g = (config_getval_msg_t *)calloc(1, sizeof(config_getval_msg_t));
    strncpy(g->cp.component, comp, CONFIG_MSG_COMPONENT_LENGTH);
    strncpy(g->cp.path, path, CONFIG_MSG_PATH_LENGTH);
    FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_CONFIGMANAGER,
							  MSG_CONFIG_GET_VALUE,
							  g, sizeof(config_getval_msg_t));
    c->enqueue(omsg);
    omsg->unref();
    c->wait(FAWKES_CID_CONFIGMANAGER);

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


/** Set new value in configuration of type float
 * @param comp component
 * @param path path to value
 * @param f new value
 */
void
NetworkConfiguration::set_float(const char *comp, const char *path, float f)
{
  mutex->lock();
  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_CONFIGMANAGER,
							MSG_CONFIG_SET_FLOAT,
							sizeof(config_float_value_msg_t));
  config_float_value_msg_t *fm = omsg->msg<config_float_value_msg_t>();
  strncpy(fm->cp.component, comp, CONFIG_MSG_COMPONENT_LENGTH);
  strncpy(fm->cp.path, path, CONFIG_MSG_COMPONENT_LENGTH);
  fm->f = f;
  c->enqueue(omsg);
  omsg->unref();
  c->wait(FAWKES_CID_CONFIGMANAGER);
  if ( ! mirror && (msg != NULL) ) {
    msg->unref();
    msg = NULL;
  }
  mutex->unlock();
}


/** Set new value in configuration of type unsigned int
 * @param comp component
 * @param path path to value
 * @param uint new value
 */
void
NetworkConfiguration::set_uint(const char *comp, const char *path, unsigned int uint)
{
  mutex->lock();
  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_CONFIGMANAGER,
							MSG_CONFIG_SET_UINT,
							sizeof(config_uint_value_msg_t));
  config_uint_value_msg_t *m = omsg->msg<config_uint_value_msg_t>();
  strncpy(m->cp.component, comp, CONFIG_MSG_COMPONENT_LENGTH);
  strncpy(m->cp.path, path, CONFIG_MSG_COMPONENT_LENGTH);
  m->u = uint;
  c->enqueue(omsg);
  omsg->unref();
  c->wait(FAWKES_CID_CONFIGMANAGER);
  if ( ! mirror && (msg != NULL) ) {
    msg->unref();
    msg = NULL;
  }
  mutex->unlock();
}


/** Set new value in configuration of type int
 * @param comp component
 * @param path path to value
 * @param i new value
 */
void
NetworkConfiguration::set_int(const char *comp, const char *path, int i)
{
  mutex->lock();
  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_CONFIGMANAGER,
							MSG_CONFIG_SET_INT,
							sizeof(config_int_value_msg_t));
  config_int_value_msg_t *m = omsg->msg<config_int_value_msg_t>();
  strncpy(m->cp.component, comp, CONFIG_MSG_COMPONENT_LENGTH);
  strncpy(m->cp.path, path, CONFIG_MSG_COMPONENT_LENGTH);
  m->i = i;
  c->enqueue(omsg);
  omsg->unref();
  c->wait(FAWKES_CID_CONFIGMANAGER);
  if ( ! mirror && (msg != NULL) ) {
    msg->unref();
    msg = NULL;
  }
  mutex->unlock();
}


/** Set new value in configuration of type bool
 * @param comp component
 * @param path path to value
 * @param b new value
 */
void
NetworkConfiguration::set_bool(const char *comp, const char *path, bool b)
{
  mutex->lock();
  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_CONFIGMANAGER,
							MSG_CONFIG_SET_BOOL,
							sizeof(config_bool_value_msg_t));
  config_bool_value_msg_t *m = omsg->msg<config_bool_value_msg_t>();
  strncpy(m->cp.component, comp, CONFIG_MSG_COMPONENT_LENGTH);
  strncpy(m->cp.path, path, CONFIG_MSG_COMPONENT_LENGTH);
  m->b = (b ? 1 : 0);
  c->enqueue(omsg);
  omsg->unref();
  c->wait(FAWKES_CID_CONFIGMANAGER);
  if ( ! mirror && (msg != NULL) ) {
    msg->unref();
    msg = NULL;
  }
  mutex->unlock();
}


/** Set new value in configuration of type string
 * @param comp component
 * @param path path to value
 * @param s new value
 * @param s_length length in bytes of s
 */
void
NetworkConfiguration::set_string(const char *comp, const char *path,
				 const char *s, unsigned int s_length)
{
  mutex->lock();
  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_CONFIGMANAGER,
							MSG_CONFIG_SET_STRING,
							sizeof(config_string_value_msg_t));
  config_string_value_msg_t *m = omsg->msg<config_string_value_msg_t>();
  strncpy(m->cp.component, comp, CONFIG_MSG_COMPONENT_LENGTH);
  strncpy(m->cp.path, path, CONFIG_MSG_COMPONENT_LENGTH);
  strncpy(m->s, s, CONFIG_MSG_MAX_STRING_LENGTH);
  c->enqueue(omsg);
  omsg->unref();
  c->wait(FAWKES_CID_CONFIGMANAGER);
  if ( ! mirror && (msg != NULL) ) {
    msg->unref();
    msg = NULL;
  }
  mutex->unlock();
}


/** Set new value in configuration of type string
 * @param comp component
 * @param path path to value
 * @param s new value
 */
void
NetworkConfiguration::set_string(const char *comp, const char *path, std::string s)
{
  set_string(comp, path, s.c_str(), s.length());
}


/** Erase the given value from the configuration. It is not an error if the value does
 * not exists before deletion.
 * @param comp component
 * @param path path to value
 */
void
NetworkConfiguration::erase(const char *comp, const char *path)
{
  mutex->lock();
  FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_CONFIGMANAGER,
							MSG_CONFIG_ERASE_VALUE,
							sizeof(config_erase_value_msg_t));
  printf("Message generated, size: %lu, should be: %lu\n", omsg->payload_size(), sizeof(config_erase_value_msg_t));
  config_erase_value_msg_t *m = omsg->msg<config_erase_value_msg_t>();
  strncpy(m->cp.component, comp, CONFIG_MSG_COMPONENT_LENGTH);
  strncpy(m->cp.path, path, CONFIG_MSG_COMPONENT_LENGTH);
  c->enqueue(omsg);
  omsg->unref();
  c->wait(FAWKES_CID_CONFIGMANAGER);
  if ( ! mirror && (msg != NULL) ) {
    msg->unref();
    msg = NULL;
  }
  mutex->unlock();
}


/** We are no longer registered in Fawkes network client.
 * Ignored.
 */
void
NetworkConfiguration::deregistered()
{
}


/** Inbound message received.
 * @param m message
 */
void
NetworkConfiguration::inboundReceived(FawkesNetworkMessage *m)
{

  if ( m->cid() == FAWKES_CID_CONFIGMANAGER ) {

    // printf("Received message of type %u\n", m->msgid());

    if ( mirror ) {
      switch (m->msgid()) {
      case MSG_CONFIG_END_OF_VALUES:
	// add all change handlers
	for (chit = change_handlers.begin(); chit != change_handlers.end(); ++chit) {
	  for (cit = (*chit).second.begin(); cit != (*chit).second.end(); ++cit) {
	    mirror_config->add_change_handler(*cit);
	  }
	}
	mutex->unlock();
	break;

      case MSG_CONFIG_VALUE_ERASED:
	try {
	  config_value_erased_msg_t *em = m->msg<config_value_erased_msg_t>();
	  mirror_config->erase(em->cp.component, em->cp.path);
	} catch (Exception &e) {
	  // Just ignore silently
	  printf("NetworkConfiguration[mirroring]::inboundReceived: erasing failed");
	}
	break;

      case MSG_CONFIG_FLOAT_VALUE:
	try {
	  config_float_value_msg_t *fm = m->msg<config_float_value_msg_t>();
	  mirror_config->set_float(fm->cp.component, fm->cp.path, fm->f);
	} catch (TypeMismatchException &e) {
	  // Just ignore silently
	  printf("NetworkConfiguration[mirroring]::inboundReceived: invalid float received");
	}
	break;

      case MSG_CONFIG_UINT_VALUE:
	try {
	  config_uint_value_msg_t *um = m->msg<config_uint_value_msg_t>();
	  mirror_config->set_uint(um->cp.component, um->cp.path, um->u);
	} catch (TypeMismatchException &e) {
	  // Just ignore silently
	  printf("NetworkConfiguration[mirroring]::inboundReceived: invalid uint received");
	}
	break;

      case MSG_CONFIG_INT_VALUE:
	try {
	  config_int_value_msg_t *im = m->msg<config_int_value_msg_t>();
	  mirror_config->set_int(im->cp.component, im->cp.path, im->i);
	} catch (TypeMismatchException &e) {
	  // Just ignore silently
	  printf("NetworkConfiguration[mirroring]::inboundReceived: invalid int received");
	}
	break;

      case MSG_CONFIG_BOOL_VALUE:
	try {
	  config_bool_value_msg_t *bm = m->msg<config_bool_value_msg_t>();
	  mirror_config->set_bool(bm->cp.component, bm->cp.path, (bm->b != 0));
	} catch (TypeMismatchException &e) {
	  // Just ignore silently
	  printf("NetworkConfiguration[mirroring]::inboundReceived: invalid bool received");
	}
	break;

      case MSG_CONFIG_STRING_VALUE:
	try {
	  config_string_value_msg_t *sm = m->msg<config_string_value_msg_t>();
	  mirror_config->set_string(sm->cp.component, sm->cp.path, sm->s, CONFIG_MSG_MAX_STRING_LENGTH);
	} catch (TypeMismatchException &e) {
	  // Just ignore silently
	  printf("NetworkConfiguration[mirroring]::inboundReceived: invalid string received");
	}
	break;
      }
    } else {
      msg = m;
      msg->ref();
    }
  }
}


/** Add a configuration change handler.
 * The added handler is called whenever a value changes and the handler
 * desires to get notified for the given component.
 * @param h configuration change handler
 */
void
NetworkConfiguration::add_change_handler(ConfigurationChangeHandler *h)
{
  const char *c = h->configMonitorComponent();
  if ( c == NULL ) {
    c = "";
  }
  change_handlers[c].push_back(h);
  change_handlers[c].sort();
  change_handlers[c].unique();
  if ( mirror ) {
    mirror_config->add_change_handler(h);
  }
}


/** Remove a configuration change handler.
 * The handler is removed from the change handler list and no longer called on
 * config changes.
 * @param h configuration change handler
 */
void
NetworkConfiguration::rem_change_handler(ConfigurationChangeHandler *h)
{
  const char *c = h->configMonitorComponent();
  if ( c == NULL ) {
    c = "";
  }
  if ( change_handlers.find(c) != change_handlers.end() ) {
    remove(change_handlers[c].begin(), change_handlers[c].end(), h);
  }
  if ( mirror ) {
    mirror_config->rem_change_handler(h);
  }
}


/** Enable or disable mirror mode.
 * If you enable mirror mode, the config is copied over in the background and the
 * configuration is locked until that is finished.
 * @param mirror true to enable mirror mode, false to disable
 */
void
NetworkConfiguration::setMirrorMode(bool mirror)
{
  if ( mirror ) {
    if ( ! this->mirror ) {
      this->mirror = true;
      // Create local temporary database
      tmp_volatile = (char *)malloc(L_tmpnam);
      tmp_default  = (char *)malloc(L_tmpnam);
      if ( (tmpnam(tmp_volatile) == NULL) ||
	   (tmpnam(tmp_default) == NULL) ) {
	free(tmp_volatile);
	free(tmp_default);
	this->mirror = false;
	throw CannotEnableMirroringException();
      }

      mirror_config = new SQLiteConfiguration();
      mirror_config->load(tmp_volatile, tmp_default);

      // subscribe
      FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_CONFIGMANAGER,
							    MSG_CONFIG_SUBSCRIBE);
      c->enqueue(omsg);
      omsg->unref();

      // unlocked after all data has been received once
      mutex->lock();
    }
  } else {
    if ( this->mirror ) {
      this->mirror = false;
      // unsubscribe
      FawkesNetworkMessage *omsg = new FawkesNetworkMessage(FAWKES_CID_CONFIGMANAGER,
							    MSG_CONFIG_UNSUBSCRIBE);
      c->enqueue(omsg);
      omsg->unref();

      // delete local temporary mirror database
      delete mirror_config;
      unlink(tmp_volatile);
      unlink(tmp_default);
      free(tmp_volatile);
      free(tmp_default);
    }
  }
}


/** Lock the config.
 * No further changes or queries can be executed on the configuration and will block until
 * the config is unlocked.
 */
void
NetworkConfiguration::lock()
{
  mutex->lock();
}


/** Try to lock the config.
 * @see Configuration::lock()
 * @return true, if the lock has been aquired, false otherwise
 */
bool
NetworkConfiguration::tryLock()
{
  return mutex->tryLock();
}


/** Unlock the config.
 * Modifications and queries are possible again.
 */
void
NetworkConfiguration::unlock()
{
  mutex->unlock();
}


/** Iterator for all values.
 * Returns an iterator that can be used to iterate over all values in the current
 * configuration.
 * @return iterator over all values
 */
Configuration::ValueIterator *
NetworkConfiguration::iterator()
{
  if ( mirror ) {
    return mirror_config->iterator();
  } else {
    return new NetConfValueIterator();
  }
}


/** Iterator with search results.
 * Returns an iterator that can be used to iterate over the search results. All values
 * whose component and path start with the given strings are returned.
 * A call like
 * @code
 *   config->search("", "");
 * @endcode
 * is effectively the same as a call to iterator().
 * @param component start of component
 * @param path start of path
 * @return iterator to search results
 */
Configuration::ValueIterator *
NetworkConfiguration::search(const char *component, const char *path)
{
  if ( mirror ) {
    return mirror_config->search(component, path);
  } else {
    return new NetConfValueIterator();
  }
}


/** @class NetworkConfiguration::NetConfValueIterator config/netconf.h
 * Network configuration value iterator.
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
  _component = NULL;
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
  _component = NULL;
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
  _component = NULL;
  _path = NULL;

  if ( (m->cid() == FAWKES_CID_CONFIGMANAGER) &&
       (m->msgid() >= MSG_CONFIG_VALUE_BEGIN) &&
       (m->msgid() <= MSG_CONFIG_VALUE_END) &&
       (m->payload_size() > sizeof(config_descriptor_t)) ) {
    msg = m;
    msg->ref();
    // extract component and path
    // all messages start with config_descriptor!
    _component = (char *)malloc(CONFIG_MSG_COMPONENT_LENGTH + 1);
    _path      = (char *)malloc(CONFIG_MSG_PATH_LENGTH + 1);
    _component[CONFIG_MSG_COMPONENT_LENGTH] = 0;
    _path[CONFIG_MSG_PATH_LENGTH] = 0;
    config_descriptor_t *cd = (config_descriptor_t *)msg->payload();
    strncpy(_component, cd->component, CONFIG_MSG_COMPONENT_LENGTH);
    strncpy(_path, cd->path, CONFIG_MSG_PATH_LENGTH);
  } else {
    // invalid value, maybe component::path does not exist!
  }
}


/** Destructor */
NetworkConfiguration::NetConfValueIterator::~NetConfValueIterator()
{
  delete i;
  if ( msg != NULL )         msg->unref();
  if ( _component != NULL )  free(_component);
  if ( _path != NULL)        free(_path);
}


/* Check if there is another element and advance to this if possible.
 * This advances to the next element, if there is one.
 * @return true, if another element has been reached, false otherwise
 */
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


/** Check if the current element is valid.
 * This is much like the classic end element for iterators. If the iterator is
 * invalid there all subsequent calls to next() shall fail.
 * @return true, if the iterator is still valid, false otherwise
 */
bool
NetworkConfiguration::NetConfValueIterator::valid()
{
  return ( (i != NULL) || (msg != NULL) );
}


/** Component of value.
 * @return component of value.
 */
const char *
NetworkConfiguration::NetConfValueIterator::component()
{
  if ( (i == NULL) ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access component on invalid iterator");
    } else {
      return _component;
    }
  } else {
    return i->component();
  }
}


/** Path of value.
 * @return path of value
 */
const char *
NetworkConfiguration::NetConfValueIterator::path()
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


/** Type of value.
 * @return string representation of value type.
 */
const char *
NetworkConfiguration::NetConfValueIterator::type()
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


/** Check if current value is a float.
 * @return true, if value is a float, false otherwise
 */
bool
NetworkConfiguration::NetConfValueIterator::is_float()
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


/** Check if current value is a unsigned int.
 * @return true, if value is a unsigned int, false otherwise
 */
bool
NetworkConfiguration::NetConfValueIterator::is_uint()
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


/** Check if current value is a int.
 * @return true, if value is a int, false otherwise
 */
bool
NetworkConfiguration::NetConfValueIterator::is_int()
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


/** Check if current value is a bool.
 * @return true, if value is a bool, false otherwise
 */
bool
NetworkConfiguration::NetConfValueIterator::is_bool()
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


/** Check if current value is a string.
 * @return true, if value is a string, false otherwise
 */
bool
NetworkConfiguration::NetConfValueIterator::is_string()
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


/** Get float value.
 * @return value
 */
float
NetworkConfiguration::NetConfValueIterator::get_float()
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on invalid iterator");
    }
    if (msg->msgid() == MSG_CONFIG_FLOAT_VALUE) {
      config_float_value_msg_t *fm = msg->msg<config_float_value_msg_t>();
      return fm->f;
    } else {
      throw TypeMismatchException("NetConfValueIterator::get_float: type mismatch");
    }
  } else {
    return i->get_float();
  }
}


/** Get unsigned int value.
 * @return value
 */
unsigned int
NetworkConfiguration::NetConfValueIterator::get_uint()
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on invalid iterator");
    }
    if (msg->msgid() == MSG_CONFIG_UINT_VALUE) {
      config_uint_value_msg_t *um = msg->msg<config_uint_value_msg_t>();
      return um->u;
    } else {
      throw TypeMismatchException("NetConfValueIterator::get_uint: type mismatch");
    }
  } else {
    return i->get_int();
  }
}


/** Get int value.
 * @return value
 */
int
NetworkConfiguration::NetConfValueIterator::get_int()
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on invalid iterator");
    }
    if (msg->msgid() == MSG_CONFIG_INT_VALUE) {
      config_int_value_msg_t *im = msg->msg<config_int_value_msg_t>();
      return im->i;
    } else {
      throw TypeMismatchException("NetConfValueIterator::get_int: type mismatch");
    }
  } else {
    return i->get_int();
  }
}


/** Get bool value.
 * @return value
 */
bool
NetworkConfiguration::NetConfValueIterator::get_bool()
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on invalid iterator");
    }
    if (msg->msgid() == MSG_CONFIG_BOOL_VALUE) {
      config_bool_value_msg_t *bm = msg->msg<config_bool_value_msg_t>();
      return (bm->b != 0);
    } else {
      throw TypeMismatchException("NetConfValueIterator::get_bool: type mismatch");
    }
  } else {
    return i->get_bool();
  }
}


/** Get string value.
 * @return value
 */
std::string
NetworkConfiguration::NetConfValueIterator::get_string()
{
  if ( i == NULL ) {
    if ( msg == NULL ) {
      throw NullPointerException("You may not access value methods on invalid iterator");
    }
    if (msg->msgid() == MSG_CONFIG_STRING_VALUE) {
      config_string_value_msg_t *sm = msg->msg<config_string_value_msg_t>();
      char tmp[CONFIG_MSG_MAX_STRING_LENGTH + 1];
      tmp[CONFIG_MSG_MAX_STRING_LENGTH] = 0;
      strncpy(tmp, sm->s, CONFIG_MSG_MAX_STRING_LENGTH);
      return tmp;
    } else {
      throw TypeMismatchException("NetConfValueIterator::get_string: type mismatch");
    }
  } else {
    return i->get_string();
  }
}
