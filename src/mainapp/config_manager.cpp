
/***************************************************************************
 *  config_manager.cpp - Fawkes configuration manager
 *
 *  Generated: Sat Jan 06 22:55:03 2007
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

#include <mainapp/config_manager.h>
#include <config/net_messages.h>
#include <utils/logging/liblogger.h>

#include <netcomm/fawkes/component_ids.h>
#include <netcomm/fawkes/hub.h>
#include <config/config.h>

#include <cstring>

/** @class FawkesConfigManager mainapp/config_manager.h
 * Fawkes Configuration Manager.
 * This class provides a manager for the configuration used in fawkes.
 * This is mainly used to allow modification of config values over the network.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config configuration, loaded and ready to be used for getting and
 * setting values
 */
FawkesConfigManager::FawkesConfigManager(Configuration *config)
  : FawkesNetworkHandler(FAWKES_CID_CONFIGMANAGER)
{
  this->config = config;
  mutex = new Mutex();
  config->add_change_handler(this);
}


/** Destructor. */
FawkesConfigManager::~FawkesConfigManager()
{
  config->rem_change_handler(this);
  inbound_queue.clear();
  delete mutex;
}


/** Set Fawkes network hub.
 * The hub will be used for network communication. The FawkesConfigManager
 * is automatically added as handler to the hub for config messages.
 * @param hub Fawkes network hub
 */
void
FawkesConfigManager::set_hub(FawkesNetworkHub *hub)
{
  this->hub = hub;
  hub->add_handler( this );
}


/** Send invalid value message.
 * @param clid client ID
 * @param component component
 * @param path path
 */
void
FawkesConfigManager::send_inv_value(unsigned int clid,
				    const char *component, const char *path)
{
  config_invval_msg_t *r = prepare_msg<config_invval_msg_t>(component, path);
  hub->send(clid, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_INV_VALUE, r, sizeof(config_invval_msg_t));
}


/** Send value.
 * @param clid client ID
 * @param i value
 */
void
FawkesConfigManager::send_value(unsigned int clid, Configuration::ValueIterator *i)
{
  if ( i->is_float() ) {
    try {
      config_float_value_msg_t *r = prepare_msg<config_float_value_msg_t>(i->component(), i->path());
      r->f = i->get_float();
      hub->send(clid, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_FLOAT_VALUE, r, sizeof(config_float_value_msg_t));
    } catch (Exception &e) {
      LibLogger::log_warn("FawkesConfigManager",
			  "send_value: Value %s::%s could not be sent\n",
			  i->component(), i->path());
      LibLogger::log_warn("FawkesConfigManager", e);
    }
  } else if ( i->is_uint() ) {
    try {
      config_uint_value_msg_t *r = prepare_msg<config_uint_value_msg_t>(i->component(), i->path());
      r->u = i->get_uint();
      hub->send(clid, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_UINT_VALUE, r, sizeof(config_uint_value_msg_t));
    } catch (Exception &e) {
      LibLogger::log_warn("FawkesConfigManager",
			  "send_value: Value %s::%s could not be sent\n",
			  i->component(), i->path());
      LibLogger::log_warn("FawkesConfigManager", e);
    }
  } else if ( i->is_int() ) {
    try {
      config_int_value_msg_t *r = prepare_msg<config_int_value_msg_t>(i->component(), i->path());
      r->i = i->get_int();
      hub->send(clid, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_INT_VALUE, r, sizeof(config_int_value_msg_t));
    } catch (Exception &e) {
      LibLogger::log_warn("FawkesConfigManager",
			  "send_value: Value %s::%s could not be sent\n",
			  i->component(), i->path());
      LibLogger::log_warn("FawkesConfigManager", e);
    }
  } else if ( i->is_bool() ) {
    try {
      config_bool_value_msg_t *r = prepare_msg<config_bool_value_msg_t>(i->component(), i->path());
      r->b = (i->get_bool() ? 1 : 0);
      hub->send(clid, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_BOOL_VALUE, r, sizeof(config_bool_value_msg_t));
    } catch (Exception &e) {
      LibLogger::log_warn("FawkesConfigManager",
			  "send_value: Value %s::%s could not be sent\n",
			  i->component(), i->path());
      LibLogger::log_warn("FawkesConfigManager", e);
    }
  } else if ( i->is_string() ) {
    try {
      config_string_value_msg_t *r = prepare_msg<config_string_value_msg_t>(i->component(), i->path());
      strncpy(r->s, i->get_string().c_str(), CONFIG_MSG_MAX_STRING_LENGTH);
      hub->send(clid, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_STRING_VALUE, r, sizeof(config_string_value_msg_t));
    } catch (Exception &e) {
      LibLogger::log_warn("FawkesConfigManager",
			  "send_value: Value %s::%s could not be sent\n",
			  i->component(), i->path());
      LibLogger::log_warn("FawkesConfigManager", e);
    }
  }
}


/** Process all network messages that have been received. */
void
FawkesConfigManager::process_after_loop()
{
  inbound_queue.lock();

  while ( ! inbound_queue.empty() ) {
    FawkesNetworkMessage *msg = inbound_queue.front();

    // printf("Received message of type %u\n", msg->msgid());

    if (msg->msgid() == MSG_CONFIG_SUBSCRIBE) {

      subscribers.push_back_locked(msg->clid());
      subscribers.sort();
      subscribers.unique();

      config->lock();
      Configuration::ValueIterator *i = config->iterator();
      while ( i->next() ) {
	send_value(msg->clid(), i);
      }
      delete i;
      config->unlock();
      hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_END_OF_VALUES);

    } else if ( msg->msgid() == MSG_CONFIG_ERASE_VALUE ) {
      try {
	config_erase_value_msg_t *m = msg->msg<config_erase_value_msg_t>();
	char component[CONFIG_MSG_COMPONENT_LENGTH + 1];
	char path[CONFIG_MSG_PATH_LENGTH + 1];
	component[CONFIG_MSG_COMPONENT_LENGTH] = 0;
	path[CONFIG_MSG_PATH_LENGTH] = 0;
	strncpy(component, m->cp.component, CONFIG_MSG_COMPONENT_LENGTH);
	strncpy(path, m->cp.path, CONFIG_MSG_PATH_LENGTH);

	config->erase(component, path);

	config_value_erased_msg_t *r = prepare_msg<config_value_erased_msg_t>(component, path);
	hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_VALUE_ERASED,
		  r, sizeof(config_value_erased_msg_t));

      } catch (Exception &e) {
	send_inv_value(msg->clid(), "?", "?");
	e.append("Failed to erase value");
	LibLogger::log_warn("FawkesConfigManager", "Failed to erase value");
	LibLogger::log_warn("FawkesConfigManager", e);
      }

    } else if ( (msg->msgid() >= MSG_CONFIG_GET_BEGIN) &&
	 (msg->msgid() <= MSG_CONFIG_GET_END) ) {

      if ( msg->payload_size() != sizeof(config_getval_msg_t) ) {
	LibLogger::log_warn("FawkesConfigManager",
			    "CONFIG_GET_FLOAT: invalid payload size "
#if __WORDSIZE == 64
			    "(received %u instead of %u bytes",
#else
			    "(received %lu instead of %lu bytes",
#endif
			    msg->payload_size(), sizeof(config_getval_msg_t));
      } else {
	config_getval_msg_t *m = (config_getval_msg_t *)msg->payload();
	char component[CONFIG_MSG_COMPONENT_LENGTH + 1];
	char path[CONFIG_MSG_PATH_LENGTH + 1];
	component[CONFIG_MSG_COMPONENT_LENGTH] = 0;
	path[CONFIG_MSG_PATH_LENGTH] = 0;
	strncpy(component, m->cp.component, CONFIG_MSG_COMPONENT_LENGTH);
	strncpy(path, m->cp.path, CONFIG_MSG_PATH_LENGTH);

	switch (msg->msgid()) {
	case MSG_CONFIG_GET_FLOAT:
	  try {
	    float f = config->get_float(component, path);
	    config_float_value_msg_t *r = prepare_msg<config_float_value_msg_t>(component, path);
	    r->f = f;
	    hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_FLOAT_VALUE,
		      r, sizeof(config_float_value_msg_t));
	  } catch (Exception &e) {
	    send_inv_value(msg->clid(), component, path);
	    LibLogger::log_warn("FawkesConfigManager",
				"get float: Value %s::%s could not be found\n",
				component, path);
	    LibLogger::log_warn("FawkesConfigManager", e);
	  }
	  break;

	case MSG_CONFIG_GET_UINT:
	  try {
	    unsigned int u = config->get_uint(component, path);
	    config_uint_value_msg_t *r = prepare_msg<config_uint_value_msg_t>(component, path);
	    r->u = u;
	    hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_UINT_VALUE,
		      r, sizeof(config_uint_value_msg_t));
	  } catch (Exception &e) {
	    send_inv_value(msg->clid(), component, path);
	    LibLogger::log_warn("FawkesConfigManager",
				"get uint: Value %s::%s could not be found\n",
				component, path);
	    LibLogger::log_warn("FawkesConfigManager", e);
	  }
	  break;

	case MSG_CONFIG_GET_INT:
	  try {
	    int i = config->get_int(component, path);
	    config_int_value_msg_t *r = prepare_msg<config_int_value_msg_t>(component, path);
	    r->i = i;
	    hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_INT_VALUE,
		      r, sizeof(config_int_value_msg_t));
	  } catch (Exception &e) {
	    send_inv_value(msg->clid(), component, path);
	    LibLogger::log_warn("FawkesConfigManager",
				"get int: Value %s::%s could not be found\n",
				component, path);
	    LibLogger::log_warn("FawkesConfigManager", e);
	  }
	  break;

	case MSG_CONFIG_GET_BOOL:
	  try {
	    bool b = config->get_bool(component, path);
	    config_bool_value_msg_t *r = prepare_msg<config_bool_value_msg_t>(component, path);
	    r->b = b;
	    hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_BOOL_VALUE,
		      r, sizeof(config_bool_value_msg_t));
	  } catch (Exception &e) {
	    send_inv_value(msg->clid(), component, path);
	    LibLogger::log_warn("FawkesConfigManager",
				"get bool: Value %s::%s could not be found\n",
				component, path);
	    LibLogger::log_warn("FawkesConfigManager", e);
	  }
	  break;

	case MSG_CONFIG_GET_STRING:
	  try {
	    std::string s = config->get_string(component, path);
	    config_string_value_msg_t *r = prepare_msg<config_string_value_msg_t>(component, path);
	    strncpy(r->s, s.c_str(), CONFIG_MSG_MAX_STRING_LENGTH);
	    hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_STRING_VALUE,
		      r, sizeof(config_string_value_msg_t));
	  } catch (Exception &e) {
	    send_inv_value(msg->clid(), component, path);
	    LibLogger::log_warn("FawkesConfigManager",
				"get string: Value %s::%s could not be found\n",
				component, path);
	    LibLogger::log_warn("FawkesConfigManager", e);
	  }
	  break;

	case MSG_CONFIG_GET_VALUE:
	  try {
	    Configuration::ValueIterator *i = config->get_value(component, path);
	    if ( i->next() ) {
	      send_value(msg->clid(), i);
	    } else {
	      send_inv_value(msg->clid(), component, path);
	    }
	    delete i;
	  } catch (ConfigurationException &e) {
	    LibLogger::log_warn("FawkesConfigManager",
				"get value: Value %s::%s could not be found\n",
				component, path);
	    LibLogger::log_warn("FawkesConfigManager", e);
	  }
	  break;

	}
      }
    } else if ( (msg->msgid() >= MSG_CONFIG_SET_BEGIN) &&
		(msg->msgid() <= MSG_CONFIG_SET_END) ) {
      
      char component[CONFIG_MSG_COMPONENT_LENGTH + 1];
      char path[CONFIG_MSG_PATH_LENGTH + 1];
      if ( msg->payload_size() < sizeof(config_descriptor_t)) {
	LibLogger::log_warn("FawkesConfigManager",
			    "inbound set: payload is too small"
#if __WORDSIZE == 64
			    "(%u is less than %u bytes",
#else
			    "(%lu is less than %lu bytes",
#endif
			    msg->payload_size(), sizeof(config_descriptor_t));
	send_inv_value(msg->clid(), "?", "?");
      } else {
	config_descriptor_t *d = (config_descriptor_t *)msg->payload();
	component[CONFIG_MSG_COMPONENT_LENGTH] = 0;
	path[CONFIG_MSG_PATH_LENGTH] = 0;
	strncpy(component, d->component, CONFIG_MSG_COMPONENT_LENGTH);
	strncpy(path, d->path, CONFIG_MSG_PATH_LENGTH);

	switch (msg->msgid()) {
	case MSG_CONFIG_SET_FLOAT:
	case MSG_CONFIG_SET_DEFAULT_FLOAT:
	  try {
	    config_float_value_msg_t *m = msg->msg<config_float_value_msg_t>();
	    if ( msg->msgid() == MSG_CONFIG_SET_FLOAT ) {
	      config->set_float(component, path, m->f);
	    } else {
	      config->set_default_float(component, path, m->f);
	    }
	    float f = config->get_float(component, path);
	    config_float_value_msg_t *r = prepare_msg<config_float_value_msg_t>(component, path);
	    r->f = f;
	    hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_FLOAT_VALUE,
		      r, sizeof(config_float_value_msg_t));
	  } catch (Exception &e) {
	    send_inv_value(msg->clid(), component, path);
	    LibLogger::log_warn("FawkesConfigManager",
				"set float: Value %s::%s could not be set\n",
				component, path);
	    LibLogger::log_warn("FawkesConfigManager", e);
	  }
	  break;
	
	case MSG_CONFIG_SET_UINT:
	case MSG_CONFIG_SET_DEFAULT_UINT:
	  try {
	    config_uint_value_msg_t *m = msg->msg<config_uint_value_msg_t>();
	    if ( msg->msgid() == MSG_CONFIG_SET_UINT ) {
	      config->set_uint(component, path, m->u);
	    } else {
	      config->set_default_uint(component, path, m->u);
	    }
	    unsigned int u = config->get_uint(component, path);
	    config_uint_value_msg_t *r = prepare_msg<config_uint_value_msg_t>(component, path);
	    r->u = u;
	    hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_UINT_VALUE,
		      r, sizeof(config_uint_value_msg_t));
	  } catch (Exception &e) {
	    send_inv_value(msg->clid(), component, path);
	    LibLogger::log_warn("FawkesConfigManager",
				"set uint: Value %s::%s could not be set\n",
				component, path);
	    LibLogger::log_warn("FawkesConfigManager", e);
	  }
	  break;
	
	case MSG_CONFIG_SET_INT:
	case MSG_CONFIG_SET_DEFAULT_INT:
	  try {
	    config_int_value_msg_t *m = msg->msg<config_int_value_msg_t>();
	    if ( msg->msgid() == MSG_CONFIG_SET_INT ) {
	      config->set_int(component, path, m->i);
	    } else {
	      config->set_default_int(component, path, m->i);
	    }
	    int i = config->get_int(component, path);
	    config_int_value_msg_t *r = prepare_msg<config_int_value_msg_t>(component, path);
	    r->i = i;
	    hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_INT_VALUE,
		      r, sizeof(config_int_value_msg_t));
	  } catch (Exception &e) {
	    send_inv_value(msg->clid(), component, path);
	    LibLogger::log_warn("FawkesConfigManager",
				"set int: Value %s::%s could not be set\n",
				component, path);
	    LibLogger::log_warn("FawkesConfigManager", e);
	  }
	  break;
	
	case MSG_CONFIG_SET_BOOL:
	case MSG_CONFIG_SET_DEFAULT_BOOL:
	  try {
	    config_bool_value_msg_t *m = msg->msg<config_bool_value_msg_t>();
	    if ( msg->msgid() == MSG_CONFIG_SET_BOOL ) {
	      config->set_bool(component, path, (m->b != 0));
	    } else {
	      config->set_default_bool(component, path, (m->b != 0));
	    }
	    bool b = config->get_bool(component, path);
	    config_bool_value_msg_t *r = prepare_msg<config_bool_value_msg_t>(component, path);
	    r->b = (b ? 1 : 0);
	    hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_BOOL_VALUE,
		      r, sizeof(config_bool_value_msg_t));
	  } catch (Exception &e) {
	    send_inv_value(msg->clid(), component, path);
	    LibLogger::log_warn("FawkesConfigManager",
				"set bool: Value %s::%s could not be set\n",
				component, path);
	    LibLogger::log_warn("FawkesConfigManager", e);
	  }
	  break;
	
	case MSG_CONFIG_SET_STRING:
	case MSG_CONFIG_SET_DEFAULT_STRING:
	  try {
	    config_string_value_msg_t *m = msg->msg<config_string_value_msg_t>();
	    char ts[CONFIG_MSG_MAX_STRING_LENGTH + 1];
	    ts[CONFIG_MSG_MAX_STRING_LENGTH] = 0;
	    strncpy(ts, m->s, CONFIG_MSG_MAX_STRING_LENGTH);
	    std::string s = ts;
	    if ( msg->msgid() == MSG_CONFIG_SET_STRING ) {
	      config->set_string(component, path, s);
	    } else {
	      config->set_default_string(component, path, s);
	    }
	    s = config->get_string(component, path);
	    config_string_value_msg_t *r = prepare_msg<config_string_value_msg_t>(component, path);
	    strncpy(r->s, s.c_str(), CONFIG_MSG_MAX_STRING_LENGTH);
	    hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_STRING_VALUE,
		      r, sizeof(config_string_value_msg_t));
	  } catch (Exception &e) {
	    send_inv_value(msg->clid(), component, path);
	    LibLogger::log_warn("FawkesConfigManager",
				"set string: Value %s::%s could not be set\n",
				component, path);
	    LibLogger::log_warn("FawkesConfigManager", e);
	  }
	  break;
	
	}
      }
    }


    msg->unref();
    inbound_queue.pop();
  }

  inbound_queue.unlock();
}


/** Handle network message.
 * The message is put into the inbound queue and processed in processAfterLoop().
 * @param msg message
 */
void
FawkesConfigManager::handle_network_message(FawkesNetworkMessage *msg)
{
  msg->ref();
  inbound_queue.push_locked(msg);
}


/** Client connected.
 * Ignored.
 * @param clid client ID
 */
void
FawkesConfigManager::client_connected(unsigned int clid)
{
}


/** Client disconnected.
 * If the client was a subscriber it is removed.
 * @param clid client ID
 */
void
FawkesConfigManager::client_disconnected(unsigned int clid)
{
  subscribers.lock();
  remove(subscribers.begin(), subscribers.end(), clid);
  subscribers.unlock();
}


/** Tag changed.
 * Ignored.
 * @param new_tag new tag
 */
void
FawkesConfigManager::configTagChanged(const char *new_tag)
{
}


/** Configuration value changed.
 * @param component component of value
 * @param path path of value
 * @param value new value
 */
void
FawkesConfigManager::configValueChanged(const char *component, const char *path, int value)
{
  subscribers.lock();
  for (sit = subscribers.begin(); sit != subscribers.end(); ++sit) {
    try {
      config_int_value_msg_t *r = prepare_msg<config_int_value_msg_t>(component, path);
      r->i = value;
      hub->send(*sit, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_INT_VALUE,
		r, sizeof(config_int_value_msg_t));
    } catch (Exception &e) {
      LibLogger::log_warn("FawkesConfigManager",
			  "configValueChanged[int]: Value for %s::%s could not be sent "
			  "to client %u\n", component, path, *sit);
      LibLogger::log_warn("FawkesConfigManager", e);
    }
  }
  subscribers.unlock();
}


/** Configuration value changed.
 * @param component component of value
 * @param path path of value
 * @param value new value
 */
void
FawkesConfigManager::configValueChanged(const char *component, const char *path,
					unsigned int value)
{
  subscribers.lock();
  for (sit = subscribers.begin(); sit != subscribers.end(); ++sit) {
    try {
      config_uint_value_msg_t *r = prepare_msg<config_uint_value_msg_t>(component, path);
      r->u = value;
      hub->send(*sit, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_UINT_VALUE,
		r, sizeof(config_uint_value_msg_t));
    } catch (Exception &e) {
      LibLogger::log_warn("FawkesConfigManager",
			  "configValueChanged[uint]: Value for %s::%s could not be sent "
			  "to client %u\n", component, path, *sit);
      LibLogger::log_warn("FawkesConfigManager", e);
    }
  }
  subscribers.unlock();
}


/** Configuration value changed.
 * @param component component of value
 * @param path path of value
 * @param value new value
 */
void
FawkesConfigManager::configValueChanged(const char *component, const char *path,
					float value)
{
  subscribers.lock();
  for (sit = subscribers.begin(); sit != subscribers.end(); ++sit) {
    try {
      config_float_value_msg_t *r = prepare_msg<config_float_value_msg_t>(component, path);
      r->f = value;
      hub->send(*sit, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_FLOAT_VALUE,
		r, sizeof(config_float_value_msg_t));
    } catch (Exception &e) {
      LibLogger::log_warn("FawkesConfigManager",
			  "configValueChanged[float]: Value for %s::%s could not be sent "
			  "to client %u\n", component, path, *sit);
      LibLogger::log_warn("FawkesConfigManager", e);
    }
  }
  subscribers.unlock();
}


/** Configuration value changed.
 * @param component component of value
 * @param path path of value
 * @param value new value
 */
void
FawkesConfigManager::configValueChanged(const char *component, const char *path,
					bool value)
{
  subscribers.lock();
  for (sit = subscribers.begin(); sit != subscribers.end(); ++sit) {
    try {
      config_bool_value_msg_t *r = prepare_msg<config_bool_value_msg_t>(component, path);
      r->b = (value ? 1 : 0);
      hub->send(*sit, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_BOOL_VALUE,
		r, sizeof(config_bool_value_msg_t));
    } catch (Exception &e) {
      LibLogger::log_warn("FawkesConfigManager",
			  "configValueChanged[bool]: Value for %s::%s could not be sent "
			  "to client %u\n", component, path, *sit);
      LibLogger::log_warn("FawkesConfigManager", e);
    }
  }
  subscribers.unlock();
}


/** Configuration value changed.
 * @param component component of value
 * @param path path of value
 * @param value new value
 */
void
FawkesConfigManager::configValueChanged(const char *component, const char *path,
					std::string value)
{
  subscribers.lock();
  for (sit = subscribers.begin(); sit != subscribers.end(); ++sit) {
    try {
      config_string_value_msg_t *r = prepare_msg<config_string_value_msg_t>(component, path);
      strncpy(r->s, value.c_str(), CONFIG_MSG_MAX_STRING_LENGTH);
      hub->send(*sit, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_STRING_VALUE,
		r, sizeof(config_string_value_msg_t));
    } catch (Exception &e) {
      LibLogger::log_warn("FawkesConfigManager",
			  "configValueChanged[string]: Value for %s::%s could not be sent "
			  "to client %u\n", component, path, *sit);
      LibLogger::log_warn("FawkesConfigManager", e);
    }
  }
  subscribers.unlock();
}


/** Configuration value erased.
 * @param component component of value
 * @param path path of value
 */
void
FawkesConfigManager::configValueErased(const char *component, const char *path)
{
  subscribers.lock();
  for (sit = subscribers.begin(); sit != subscribers.end(); ++sit) {
    try {
      config_value_erased_msg_t *r = prepare_msg<config_value_erased_msg_t>(component, path);
      hub->send(*sit, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_VALUE_ERASED,
		r, sizeof(config_value_erased_msg_t));
    } catch (Exception &e) {
      LibLogger::log_warn("FawkesConfigManager",
			  "configValueErased: Value for %s::%s could not be sent "
			  "to client %u\n", component, path, *sit);
    }
  }
  subscribers.unlock();
}


/** Component to monitor.
 * @return NULL, monitor all components.
 */
const char *
FawkesConfigManager::configMonitorComponent()
{
  return NULL;
}
