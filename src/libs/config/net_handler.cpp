
/***************************************************************************
 *  net_handler.cpp - Fawkes configuration network handler
 *
 *  Generated: Sat Jan 06 22:55:03 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#include <config/net_handler.h>
#include <config/net_messages.h>
#include <config/net_list_content.h>
#include <logging/liblogger.h>

#include <netcomm/fawkes/component_ids.h>
#include <netcomm/fawkes/hub.h>
#include <config/config.h>

#include <algorithm>
#include <cstring>

namespace fawkes {

/** @class ConfigNetworkHandler <config/net_handler.h>
 * Fawkes Configuration Network Handler.
 * It provides access to a given config via the network.
 * This is mainly used to allow modification of config values over the network.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config configuration, loaded and ready to be used for getting and
 * setting values
 * @param hub Fawkes network hub to use for receiving and sending network
 * messages
 */
ConfigNetworkHandler::ConfigNetworkHandler(Configuration *config,
					 FawkesNetworkHub *hub)
  : Thread("ConfigNetworkHandler", Thread::OPMODE_WAITFORWAKEUP),
    FawkesNetworkHandler(FAWKES_CID_CONFIGMANAGER),
    ConfigurationChangeHandler("")
{
  __config = config;
  __hub    = hub;

  start();

  __config->add_change_handler(this);
  __hub->add_handler( this );
}


/** Destructor. */
ConfigNetworkHandler::~ConfigNetworkHandler()
{
  cancel();
  join();
  __config->rem_change_handler(this);
  __inbound_queue.clear();
}


/** Send invalid value message.
 * @param clid client ID
 * @param path path
 */
void
ConfigNetworkHandler::send_inv_value(unsigned int clid, const char *path)
{
  config_invval_msg_t *r = prepare_msg<config_invval_msg_t>(path, false);
  __hub->send(clid, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_INV_VALUE, r, sizeof(config_invval_msg_t));
}


/** Send value.
 * @param clid client ID
 * @param i value
 */
void
ConfigNetworkHandler::send_value(unsigned int clid, Configuration::ValueIterator *i)
{
  if ( i->is_float() ) {
    try {
      config_float_value_msg_t *r = prepare_msg<config_float_value_msg_t>(i->path(), i->is_default());
      r->f = i->get_float();
      __hub->send(clid, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_FLOAT_VALUE, r, sizeof(config_float_value_msg_t));
    } catch (Exception &e) {
      LibLogger::log_warn("ConfigNetworkHandler",
			  "send_value: Value %s could not be sent",
			  i->path());
      LibLogger::log_warn("ConfigNetworkHandler", e);
    }
  } else if ( i->is_uint() ) {
    try {
      config_uint_value_msg_t *r = prepare_msg<config_uint_value_msg_t>(i->path(), i->is_default());
      r->u = i->get_uint();
      __hub->send(clid, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_UINT_VALUE, r, sizeof(config_uint_value_msg_t));
    } catch (Exception &e) {
      LibLogger::log_warn("ConfigNetworkHandler",
			  "send_value: Value %s could not be sent",
			  i->path());
      LibLogger::log_warn("ConfigNetworkHandler", e);
    }
  } else if ( i->is_int() ) {
    try {
      config_int_value_msg_t *r = prepare_msg<config_int_value_msg_t>(i->path(), i->is_default());
      r->i = i->get_int();
      __hub->send(clid, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_INT_VALUE, r, sizeof(config_int_value_msg_t));
    } catch (Exception &e) {
      LibLogger::log_warn("ConfigNetworkHandler",
			  "send_value: Value %s could not be sent",
			  i->path());
      LibLogger::log_warn("ConfigNetworkHandler", e);
    }
  } else if ( i->is_bool() ) {
    try {
      config_bool_value_msg_t *r = prepare_msg<config_bool_value_msg_t>(i->path(), i->is_default());
      r->b = (i->get_bool() ? 1 : 0);
      __hub->send(clid, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_BOOL_VALUE, r, sizeof(config_bool_value_msg_t));
    } catch (Exception &e) {
      LibLogger::log_warn("ConfigNetworkHandler",
			  "send_value: Value %s could not be sent",
			  i->path());
      LibLogger::log_warn("ConfigNetworkHandler", e);
    }
  } else if ( i->is_string() ) {
    try {
      size_t sl = sizeof(config_string_value_msg_t) + i->get_string().length();
      config_string_value_msg_t *m = (config_string_value_msg_t *)calloc(1, sl);
      strncpy(m->cp.path, i->path(), CONFIG_MSG_PATH_LENGTH);
      m->cp.is_default = i->is_default() ? 1 : 0;
      m->s_length = i->get_string().length();
      strcpy(m->s, i->get_string().c_str());
      __hub->send(clid, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_STRING_VALUE, m, sl);
    } catch (Exception &e) {
      LibLogger::log_warn("ConfigNetworkHandler",
			  "send_value: Value %s could not be sent",
			  i->path());
      LibLogger::log_warn("ConfigNetworkHandler", e);
    }
  }
}


/** Process all network messages that have been received. */
void
ConfigNetworkHandler::loop()
{
  while ( ! __inbound_queue.empty() ) {
    FawkesNetworkMessage *msg = __inbound_queue.front();

    // printf("Received message of type %u\n", msg->msgid());

    if (msg->msgid() == MSG_CONFIG_SUBSCRIBE) {

      __subscribers.push_back_locked(msg->clid());
      __subscribers.sort();
      __subscribers.unique();

      __config->lock();
      ConfigListContent *content = new ConfigListContent();
      Configuration::ValueIterator *i = __config->iterator();
      while ( i->next() ) {
	if (i->is_default()) {
	  content->append(i);
	}
      }
      delete i;
      i = __config->iterator();
      while ( i->next() ) {
	if (! i->is_default()) {
	  content->append(i);
	}
      }
      delete i;
      __hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_LIST, content);
      __config->unlock();

    } else if (msg->msgid() == MSG_CONFIG_ERASE_VALUE) {
      try {
	config_erase_value_msg_t *m = msg->msg<config_erase_value_msg_t>();
	char path[CONFIG_MSG_PATH_LENGTH + 1];
	path[CONFIG_MSG_PATH_LENGTH] = 0;
	strncpy(path, m->cp.path, CONFIG_MSG_PATH_LENGTH);

	if ( m->cp.is_default == 1 ) {
	  __config->erase_default(path);
	} else {
	  __config->erase(path);
	}

	config_value_erased_msg_t *r = prepare_msg<config_value_erased_msg_t>(path, (m->cp.is_default == 1));
	__hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_VALUE_ERASED,
		    r, sizeof(config_value_erased_msg_t));

      } catch (Exception &e) {
	send_inv_value(msg->clid(), "?");
	e.append("Failed to erase value");
	LibLogger::log_warn("ConfigNetworkHandler", "Failed to erase value");
	LibLogger::log_warn("ConfigNetworkHandler", e);
      }

    } else if ( (msg->msgid() >= MSG_CONFIG_GET_BEGIN) &&
	 (msg->msgid() <= MSG_CONFIG_GET_END) ) {

      if ( msg->payload_size() != sizeof(config_getval_msg_t) ) {
	LibLogger::log_warn("ConfigNetworkHandler",
			    "CONFIG_GET_FLOAT: invalid payload size "
			    "(received %zu instead of %zu bytes",
			    msg->payload_size(), sizeof(config_getval_msg_t));
      } else {
	config_getval_msg_t *m = (config_getval_msg_t *)msg->payload();
	char path[CONFIG_MSG_PATH_LENGTH + 1];
	path[CONFIG_MSG_PATH_LENGTH] = 0;
	strncpy(path, m->cp.path, CONFIG_MSG_PATH_LENGTH);

	switch (msg->msgid()) {
	case MSG_CONFIG_GET_FLOAT:
	  try {
	    float f = __config->get_float(path);
	    bool  d = __config->is_default(path);
	    config_float_value_msg_t *r = prepare_msg<config_float_value_msg_t>(path, d);
	    r->f = f;
	    __hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_FLOAT_VALUE,
		      r, sizeof(config_float_value_msg_t));
	  } catch (Exception &e) {
	    send_inv_value(msg->clid(), path);
	    LibLogger::log_warn("ConfigNetworkHandler",
				"get float: Value %s could not be found", path);
	    LibLogger::log_warn("ConfigNetworkHandler", e);
	  }
	  break;

	case MSG_CONFIG_GET_UINT:
	  try {
	    unsigned int u = __config->get_uint(path);
	    bool  d = __config->is_default(path);
	    config_uint_value_msg_t *r = prepare_msg<config_uint_value_msg_t>(path, d);
	    r->u = u;
	    __hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_UINT_VALUE,
		      r, sizeof(config_uint_value_msg_t));
	  } catch (Exception &e) {
	    send_inv_value(msg->clid(), path);
	    LibLogger::log_warn("ConfigNetworkHandler",
				"get uint: Value %s could not be found", path);
	    LibLogger::log_warn("ConfigNetworkHandler", e);
	  }
	  break;

	case MSG_CONFIG_GET_INT:
	  try {
	    int i = __config->get_int(path);
	    bool  d = __config->is_default(path);
	    config_int_value_msg_t *r = prepare_msg<config_int_value_msg_t>(path, d);
	    r->i = i;
	    __hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_INT_VALUE,
		      r, sizeof(config_int_value_msg_t));
	  } catch (Exception &e) {
	    send_inv_value(msg->clid(), path);
	    LibLogger::log_warn("ConfigNetworkHandler",
				"get int: Value %s could not be found", path);
	    LibLogger::log_warn("ConfigNetworkHandler", e);
	  }
	  break;

	case MSG_CONFIG_GET_BOOL:
	  try {
	    bool b = __config->get_bool(path);
	    bool d = __config->is_default(path);
	    config_bool_value_msg_t *r = prepare_msg<config_bool_value_msg_t>(path, d);
	    r->b = b;
	    __hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_BOOL_VALUE,
		      r, sizeof(config_bool_value_msg_t));
	  } catch (Exception &e) {
	    send_inv_value(msg->clid(), path);
	    LibLogger::log_warn("ConfigNetworkHandler",
				"get bool: Value %s could not be found", path);
	    LibLogger::log_warn("ConfigNetworkHandler", e);
	  }
	  break;

	case MSG_CONFIG_GET_STRING:
	  try {
	    std::string s = __config->get_string(path);
	    bool  d = __config->is_default(path);
	    config_string_value_msg_t *r = prepare_string_msg<config_string_value_msg_t>(path, d, s.length());
	    strcpy(r->s, s.c_str());
	    __hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_STRING_VALUE,
		      r, sizeof(config_string_value_msg_t));
	  } catch (Exception &e) {
	    send_inv_value(msg->clid(), path);
	    LibLogger::log_warn("ConfigNetworkHandler",
				"get string: Value %s could not be found", path);
	    LibLogger::log_warn("ConfigNetworkHandler", e);
	  }
	  break;

	case MSG_CONFIG_GET_VALUE:
	  try {
	    Configuration::ValueIterator *i = __config->get_value(path);
	    if ( i->next() ) {
	      send_value(msg->clid(), i);
	    } else {
	      send_inv_value(msg->clid(), path);
	    }
	    delete i;
	  } catch (ConfigurationException &e) {
	    LibLogger::log_warn("ConfigNetworkHandler",
				"get value: Value %s could not be found", path);
	    LibLogger::log_warn("ConfigNetworkHandler", e);
	  }
	  break;

	}
      }
    } else if ( (msg->msgid() >= MSG_CONFIG_SET_BEGIN) &&
		(msg->msgid() <= MSG_CONFIG_SET_END) ) {
      
      char path[CONFIG_MSG_PATH_LENGTH + 1];
      if ( msg->payload_size() < sizeof(config_descriptor_t)) {
	LibLogger::log_warn("ConfigNetworkHandler",
			    "inbound set: payload is too small"
			    "(%zu is less than %zu bytes",
			    msg->payload_size(), sizeof(config_descriptor_t));
	send_inv_value(msg->clid(), "?");
      } else {
	config_descriptor_t *d = (config_descriptor_t *)msg->payload();
	path[CONFIG_MSG_PATH_LENGTH] = 0;
	strncpy(path, d->path, CONFIG_MSG_PATH_LENGTH);

	switch (msg->msgid()) {
	case MSG_CONFIG_SET_FLOAT:
	case MSG_CONFIG_SET_DEFAULT_FLOAT:
	  try {
	    config_float_value_msg_t *m = msg->msg<config_float_value_msg_t>();
	    if ( msg->msgid() == MSG_CONFIG_SET_FLOAT ) {
	      __config->set_float(path, m->f);
	    } else {
	      __config->set_default_float(path, m->f);
	    }
	    float f = __config->get_float(path);
	    config_float_value_msg_t *r = prepare_msg<config_float_value_msg_t>(path, (msg->msgid() == MSG_CONFIG_SET_DEFAULT_FLOAT));
	    r->f = f;
	    __hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_FLOAT_VALUE,
		      r, sizeof(config_float_value_msg_t));
	  } catch (Exception &e) {
	    send_inv_value(msg->clid(), path);
	    LibLogger::log_warn("ConfigNetworkHandler",
				"set float: Value %s could not be set", path);
	    LibLogger::log_warn("ConfigNetworkHandler", e);
	  }
	  break;
	
	case MSG_CONFIG_SET_UINT:
	case MSG_CONFIG_SET_DEFAULT_UINT:
	  try {
	    config_uint_value_msg_t *m = msg->msg<config_uint_value_msg_t>();
	    if ( msg->msgid() == MSG_CONFIG_SET_UINT ) {
	      __config->set_uint(path, m->u);
	    } else {
	      __config->set_default_uint(path, m->u);
	    }
	    unsigned int u = __config->get_uint(path);
	    config_uint_value_msg_t *r = prepare_msg<config_uint_value_msg_t>(path, (msg->msgid() == MSG_CONFIG_SET_DEFAULT_UINT));
	    r->u = u;
	    __hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_UINT_VALUE,
		      r, sizeof(config_uint_value_msg_t));
	  } catch (Exception &e) {
	    send_inv_value(msg->clid(), path);
	    LibLogger::log_warn("ConfigNetworkHandler",
				"set uint: Value %s could not be set", path);
	    LibLogger::log_warn("ConfigNetworkHandler", e);
	  }
	  break;
	
	case MSG_CONFIG_SET_INT:
	case MSG_CONFIG_SET_DEFAULT_INT:
	  try {
	    config_int_value_msg_t *m = msg->msg<config_int_value_msg_t>();
	    if ( msg->msgid() == MSG_CONFIG_SET_INT ) {
	      __config->set_int(path, m->i);
	    } else {
	      __config->set_default_int(path, m->i);
	    }
	    int i = __config->get_int(path);
	    config_int_value_msg_t *r = prepare_msg<config_int_value_msg_t>(path, (msg->msgid() == MSG_CONFIG_SET_DEFAULT_INT));
	    r->i = i;
	    __hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_INT_VALUE,
		      r, sizeof(config_int_value_msg_t));
	  } catch (Exception &e) {
	    send_inv_value(msg->clid(), path);
	    LibLogger::log_warn("ConfigNetworkHandler",
				"set int: Value %s could not be set", path);
	    LibLogger::log_warn("ConfigNetworkHandler", e);
	  }
	  break;
	
	case MSG_CONFIG_SET_BOOL:
	case MSG_CONFIG_SET_DEFAULT_BOOL:
	  try {
	    config_bool_value_msg_t *m = msg->msg<config_bool_value_msg_t>();
	    if ( msg->msgid() == MSG_CONFIG_SET_BOOL ) {
	      __config->set_bool(path, (m->b != 0));
	    } else {
	      __config->set_default_bool(path, (m->b != 0));
	    }
	    bool b = __config->get_bool(path);
	    config_bool_value_msg_t *r = prepare_msg<config_bool_value_msg_t>(path, (msg->msgid() == MSG_CONFIG_SET_DEFAULT_BOOL));
	    r->b = (b ? 1 : 0);
	    __hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_BOOL_VALUE,
		      r, sizeof(config_bool_value_msg_t));
	  } catch (Exception &e) {
	    send_inv_value(msg->clid(), path);
	    LibLogger::log_warn("ConfigNetworkHandler",
				"set bool: Value %s could not be set", path);
	    LibLogger::log_warn("ConfigNetworkHandler", e);
	  }
	  break;
	
	case MSG_CONFIG_SET_STRING:
	case MSG_CONFIG_SET_DEFAULT_STRING:
	  try {
	    config_string_value_msg_t *m = msg->msgge<config_string_value_msg_t>();
	    if ( msg->msgid() == MSG_CONFIG_SET_STRING ) {
	      __config->set_string(path, m->s);
	    } else {
	      __config->set_default_string(path, m->s);
	    }
	    std::string s = __config->get_string(path);
	    size_t s_length = s.length();
	    config_string_value_msg_t *r = prepare_string_msg<config_string_value_msg_t>(path, (msg->msgid() == MSG_CONFIG_SET_DEFAULT_STRING), s_length);
	    strcpy(r->s, s.c_str());
	    __hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_STRING_VALUE,
		      r, sizeof(config_string_value_msg_t) + s_length);
	  } catch (Exception &e) {
	    send_inv_value(msg->clid(), path);
	    LibLogger::log_warn("ConfigNetworkHandler",
				"set string: Value %s could not be set", path);
	    LibLogger::log_warn("ConfigNetworkHandler", e);
	  }
	  break;

	case MSG_CONFIG_SET_COMMENT:
	case MSG_CONFIG_SET_DEFAULT_COMMENT:
	  try {
	    config_comment_msg_t *m = msg->msgge<config_comment_msg_t>();
	    std::string s = "";
	    if ( msg->msgid() == MSG_CONFIG_SET_COMMENT ) {
	      __config->set_comment(path, m->s);
	      s = __config->get_comment(path);
	    } else {
	      __config->set_default_comment(path, m->s);
	      s = __config->get_default_comment(path);
	    }
	    size_t s_length = s.length();
	    config_comment_msg_t *r = prepare_string_msg<config_comment_msg_t>(path, (msg->msgid() == MSG_CONFIG_SET_DEFAULT_COMMENT), s_length);
	    strcpy(r->s, s.c_str());
	    __hub->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_COMMENT_VALUE,
			r, sizeof(config_comment_msg_t) + s_length);
	  } catch (Exception &e) {
	    send_inv_value(msg->clid(), path);
	    LibLogger::log_warn("ConfigNetworkHandler",
				"set comment: Value %s could not be set", path);
	    LibLogger::log_warn("ConfigNetworkHandler", e);
	  }
	  break;
	
	}
      }
    }


    msg->unref();
    __inbound_queue.pop_locked();
  }
}


/** Handle network message.
 * The message is put into the inbound queue and processed in processAfterLoop().
 * @param msg message
 */
void
ConfigNetworkHandler::handle_network_message(FawkesNetworkMessage *msg)
{
  msg->ref();
  __inbound_queue.push_locked(msg);
  wakeup();
}


/** Client connected.
 * Ignored.
 * @param clid client ID
 */
void
ConfigNetworkHandler::client_connected(unsigned int clid)
{
}


/** Client disconnected.
 * If the client was a subscriber it is removed.
 * @param clid client ID
 */
void
ConfigNetworkHandler::client_disconnected(unsigned int clid)
{
  __subscribers.lock();
  if (find(__subscribers.begin(), __subscribers.end(), clid) != __subscribers.end()) {
    LibLogger::log_warn("ConfigNetworkHandler",
			"Client %u disconnected without closing the config, removing from list of subscribers",
			clid);
    __subscribers.remove(clid);
  }
  __subscribers.unlock();
}


/** Tag changed.
 * Ignored.
 * @param new_tag new tag
 */
void
ConfigNetworkHandler::config_tag_changed(const char *new_tag)
{
}


void
ConfigNetworkHandler::config_value_changed(const Configuration::ValueIterator *v)
{
  const char *path = v->path();
  bool is_default  = v->is_default();

  __subscribers.lock();
  for (__sit = __subscribers.begin(); __sit != __subscribers.end(); ++__sit) {
    try {
      if (v->is_string()) {
	std::string value = v->get_string();
	size_t s_length = value.length();
	config_string_value_msg_t *r =
	  prepare_string_msg<config_string_value_msg_t>(path, is_default,
							s_length);
	strcpy(r->s, value.c_str());
	__hub->send(*__sit, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_STRING_VALUE,
		    r, sizeof(config_string_value_msg_t) + s_length);
      } else if (v->is_bool()) {
	bool value = v->get_bool();
	config_bool_value_msg_t *r =
	  prepare_msg<config_bool_value_msg_t>(path, is_default);
	r->b = (value ? 1 : 0);
	__hub->send(*__sit, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_BOOL_VALUE,
		    r, sizeof(config_bool_value_msg_t));
      } else if (v->is_int()) {
	int value = v->get_int();
	config_int_value_msg_t *r =
	  prepare_msg<config_int_value_msg_t>(path, is_default);
	r->i = value;
	__hub->send(*__sit, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_INT_VALUE,
		    r, sizeof(config_int_value_msg_t));
      } else if (v->is_uint()) {
	unsigned int value = v->get_uint();
	config_uint_value_msg_t *r =
	  prepare_msg<config_uint_value_msg_t>(path, is_default);
	r->u = value;
	__hub->send(*__sit, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_UINT_VALUE,
		    r, sizeof(config_uint_value_msg_t));
      } else if (v->is_float()) {
	float value = v->get_float();
	config_float_value_msg_t *r =
	  prepare_msg<config_float_value_msg_t>(path, is_default);
	r->f = value;
	__hub->send(*__sit, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_FLOAT_VALUE,
		    r, sizeof(config_float_value_msg_t));
      }

    } catch (Exception &e) {
      LibLogger::log_warn("ConfigNetworkHandler",
			  "config_value_changed: Value for %s could not be sent "
			  "to client %u", path, *__sit);
      LibLogger::log_warn("ConfigNetworkHandler", e);
    }
  }
  __subscribers.unlock();
}


void
ConfigNetworkHandler::config_comment_changed(const Configuration::ValueIterator *v)
{
  const char *path = v->path();
  bool is_default  = v->is_default();
  std::string comment = v->get_comment();
  __subscribers.lock();
  for (__sit = __subscribers.begin(); __sit != __subscribers.end(); ++__sit) {
    try {
      size_t s_length = comment.length();
      config_comment_msg_t *r =
	prepare_string_msg<config_comment_msg_t>(path, is_default, s_length);
      strcpy(r->s, comment.c_str());

      __hub->send(*__sit, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_COMMENT_VALUE,
		  r, sizeof(config_comment_msg_t) + s_length);
    } catch (Exception &e) {
      LibLogger::log_warn("ConfigNetworkHandler",
			  "config_comment_changed[string]: Value for %s could "
			  "not be sent to client %u", path, *__sit);
      LibLogger::log_warn("ConfigNetworkHandler", e);
    }
  }
  __subscribers.unlock();
}


void
ConfigNetworkHandler::config_value_erased(const char *path)
{
  __subscribers.lock();
  for (__sit = __subscribers.begin(); __sit != __subscribers.end(); ++__sit) {
    try {
      config_value_erased_msg_t *r =
	prepare_msg<config_value_erased_msg_t>(path, false);
      __hub->send(*__sit, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_VALUE_ERASED,
		  r, sizeof(config_value_erased_msg_t));
    } catch (Exception &e) {
      LibLogger::log_warn("ConfigNetworkHandler",
			  "configValueErased: Value for %s could not be sent "
			  "to client %u", path, *__sit);
    }
  }
  __subscribers.unlock();
}

} // end namespace fawkes
