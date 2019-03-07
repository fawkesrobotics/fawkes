
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

#include <config/config.h>
#include <config/net_handler.h>
#include <config/net_list_content.h>
#include <config/net_messages.h>
#include <logging/liblogger.h>
#include <netcomm/fawkes/component_ids.h>
#include <netcomm/fawkes/hub.h>

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
ConfigNetworkHandler::ConfigNetworkHandler(Configuration *config, FawkesNetworkHub *hub)
: Thread("ConfigNetworkHandler", Thread::OPMODE_WAITFORWAKEUP),
  FawkesNetworkHandler(FAWKES_CID_CONFIGMANAGER),
  ConfigurationChangeHandler("")
{
	config_ = config;
	hub_    = hub;

	start();

	config_->add_change_handler(this);
	hub_->add_handler(this);
}

/** Destructor. */
ConfigNetworkHandler::~ConfigNetworkHandler()
{
	hub_->remove_handler(this);
	cancel();
	join();
	config_->rem_change_handler(this);
	inbound_queue_.clear();
}

/** Send invalid value message.
 * @param clid client ID
 * @param path path
 */
void
ConfigNetworkHandler::send_inv_value(unsigned int clid, const char *path)
{
	config_invval_msg_t *r = prepare_msg<config_invval_msg_t>(path, false);
	hub_->send(clid, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_INV_VALUE, r, sizeof(config_invval_msg_t));
}

/** Send value.
 * @param clid client ID
 * @param i value
 */
void
ConfigNetworkHandler::send_value(unsigned int clid, const Configuration::ValueIterator *i)
{
	if (i->is_uint()) {
		try {
			uint32_t *values;
			uint16_t  num_values = i->is_list() ? i->get_list_size() : 0;
			size_t    data_size  = 0;
			void *    m          = prepare_value_msg<uint32_t>(
        i->path(), i->is_default(), i->is_list(), num_values, data_size, (void **)&values);
			if (i->is_list()) {
				std::vector<unsigned int> c_values = i->get_uints();
				for (uint16_t j = 0; j < num_values; ++j)
					values[j] = c_values[j];
			} else {
				values[0] = i->get_uint();
			}
			hub_->send(clid, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_UINT_VALUE, m, data_size);
		} catch (Exception &e) {
			LibLogger::log_warn("ConfigNetworkHandler",
			                    "send_value: Value %s could not be sent",
			                    i->path());
			LibLogger::log_warn("ConfigNetworkHandler", e);
		}
	} else if (i->is_int()) {
		try {
			int32_t *values;
			int16_t  num_values = i->is_list() ? i->get_list_size() : 0;
			size_t   data_size  = 0;
			void *   m          = prepare_value_msg<int32_t>(
        i->path(), i->is_default(), i->is_list(), num_values, data_size, (void **)&values);
			if (i->is_list()) {
				std::vector<int> c_values = i->get_ints();
				for (uint16_t j = 0; j < num_values; ++j)
					values[j] = c_values[j];
			} else {
				values[0] = i->get_int();
			}
			hub_->send(clid, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_INT_VALUE, m, data_size);
		} catch (Exception &e) {
			LibLogger::log_warn("ConfigNetworkHandler",
			                    "send_value: Value %s could not be sent",
			                    i->path());
			LibLogger::log_warn("ConfigNetworkHandler", e);
		}
	} else if (i->is_bool()) {
		try {
			int32_t *values;
			int16_t  num_values = i->is_list() ? i->get_list_size() : 0;
			size_t   data_size  = 0;
			void *   m          = prepare_value_msg<int32_t>(
        i->path(), i->is_default(), i->is_list(), num_values, data_size, (void **)&values);
			if (i->is_list()) {
				std::vector<bool> c_values = i->get_bools();
				for (uint16_t j = 0; j < num_values; ++j)
					values[j] = (c_values[j] ? 1 : 0);
			} else {
				values[0] = i->get_bool() ? 1 : 0;
			}
			hub_->send(clid, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_BOOL_VALUE, m, data_size);
		} catch (Exception &e) {
			LibLogger::log_warn("ConfigNetworkHandler",
			                    "send_value: Value %s could not be sent",
			                    i->path());
			LibLogger::log_warn("ConfigNetworkHandler", e);
		}
	} else if (i->is_float()) {
		try {
			float *  values;
			uint16_t num_values = i->is_list() ? i->get_list_size() : 0;
			size_t   data_size  = 0;
			void *   m          = prepare_value_msg<float>(
        i->path(), i->is_default(), i->is_list(), num_values, data_size, (void **)&values);
			if (i->is_list()) {
				std::vector<float> c_values = i->get_floats();
				for (uint16_t j = 0; j < num_values; ++j)
					values[j] = c_values[j];
			} else {
				values[0] = i->get_float();
			}
			hub_->send(clid, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_FLOAT_VALUE, m, data_size);
		} catch (Exception &e) {
			LibLogger::log_warn("ConfigNetworkHandler",
			                    "send_value: Value %s could not be sent",
			                    i->path());
			LibLogger::log_warn("ConfigNetworkHandler", e);
		}
	} else if (i->is_string()) {
		try {
			if (i->is_list()) {
				std::vector<std::string> s         = i->get_strings();
				size_t                   data_size = sizeof(config_descriptor_t);

				for (unsigned int j = 0; j < s.size(); ++j) {
					data_size += sizeof(config_string_value_t) + s[j].length() + 1;
				}
				void *m = calloc(1, data_size);

				config_descriptor_t *cd = (config_descriptor_t *)m;
				strncpy(cd->path, i->path(), CONFIG_MSG_PATH_LENGTH - 1);
				cd->is_default = i->is_default();
				cd->num_values = s.size();

				char *tmp = ((char *)m + sizeof(config_descriptor_t));
				for (unsigned int j = 0; j < s.size(); ++j) {
					config_string_value_t *sv         = (config_string_value_t *)tmp;
					char *                 msg_string = tmp + sizeof(config_string_value_t);
					sv->s_length                      = s[j].length();
					strcpy(msg_string, s[j].c_str());
					tmp += sizeof(config_string_value_t) + sv->s_length + 1;
				}

				hub_->send(clid, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_STRING_VALUE, m, data_size);
			} else {
				std::string s = i->get_string();
				size_t      data_size =
				  sizeof(config_descriptor_t) + sizeof(config_string_value_t) + s.length() + 1;
				void *               m  = calloc(1, data_size);
				config_descriptor_t *cd = (config_descriptor_t *)m;
				strncpy(cd->path, i->path(), CONFIG_MSG_PATH_LENGTH - 1);
				cd->is_default = i->is_default();
				cd->num_values = 0;

				config_string_value_t *sv =
				  (config_string_value_t *)((char *)m + sizeof(config_descriptor_t));
				char *msg_string = (char *)sv + sizeof(config_string_value_t);

				sv->s_length = s.length();
				strcpy(msg_string, s.c_str());

				hub_->send(clid, FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_STRING_VALUE, m, data_size);
			}
		} catch (Exception &e) {
			LibLogger::log_warn("ConfigNetworkHandler",
			                    "send_value: Value %s could not be sent",
			                    i->path());
			LibLogger::log_warn("ConfigNetworkHandler", e);
		}
	} else {
		LibLogger::log_warn("ConfigNetworkHandler", "send_value: unknown type of value %s", i->path());
	}
}

/** Process all network messages that have been received. */
void
ConfigNetworkHandler::loop()
{
	while (!inbound_queue_.empty()) {
		FawkesNetworkMessage *msg = inbound_queue_.front();

		// printf("Received message of type %u\n", msg->msgid());

		if (msg->msgid() == MSG_CONFIG_SUBSCRIBE) {
			subscribers_.push_back_locked(msg->clid());
			subscribers_.sort();
			subscribers_.unique();

			config_->lock();
			ConfigListContent *           content = new ConfigListContent();
			Configuration::ValueIterator *i       = config_->iterator();
			while (i->next()) {
				if (i->is_default()) {
					content->append(i);
				}
			}
			delete i;
			i = config_->iterator();
			while (i->next()) {
				if (!i->is_default()) {
					content->append(i);
				}
			}
			delete i;
			hub_->send(msg->clid(), FAWKES_CID_CONFIGMANAGER, MSG_CONFIG_LIST, content);
			config_->unlock();

		} else if (msg->msgid() == MSG_CONFIG_ERASE_VALUE) {
			try {
				config_erase_value_msg_t *m = msg->msg<config_erase_value_msg_t>();
				char                      path[CONFIG_MSG_PATH_LENGTH];
				path[CONFIG_MSG_PATH_LENGTH - 1] = 0;
				memcpy(path, m->cp.path, CONFIG_MSG_PATH_LENGTH);

				if (m->cp.is_default == 1) {
					config_->erase_default(path);
				} else {
					config_->erase(path);
				}

				config_value_erased_msg_t *r =
				  prepare_msg<config_value_erased_msg_t>(path, (m->cp.is_default == 1));
				hub_->send(msg->clid(),
				           FAWKES_CID_CONFIGMANAGER,
				           MSG_CONFIG_VALUE_ERASED,
				           r,
				           sizeof(config_value_erased_msg_t));

			} catch (Exception &e) {
				send_inv_value(msg->clid(), "?");
				e.append("Failed to erase value");
				LibLogger::log_warn("ConfigNetworkHandler", "Failed to erase value");
				LibLogger::log_warn("ConfigNetworkHandler", e);
			}

		} else if ((msg->msgid() >= MSG_CONFIG_GET_BEGIN) && (msg->msgid() <= MSG_CONFIG_GET_END)) {
			if (msg->payload_size() != sizeof(config_getval_msg_t)) {
				LibLogger::log_warn("ConfigNetworkHandler",
				                    "CONFIG_GET_FLOAT: invalid payload size "
				                    "(received %zu instead of %zu bytes",
				                    msg->payload_size(),
				                    sizeof(config_getval_msg_t));
			} else {
				config_getval_msg_t *m = (config_getval_msg_t *)msg->payload();
				char                 path[CONFIG_MSG_PATH_LENGTH + 1];
				path[CONFIG_MSG_PATH_LENGTH] = 0;
				strncpy(path, m->cp.path, CONFIG_MSG_PATH_LENGTH);

				switch (msg->msgid()) {
				case MSG_CONFIG_GET_FLOAT:
				case MSG_CONFIG_GET_UINT:
				case MSG_CONFIG_GET_INT:
				case MSG_CONFIG_GET_BOOL:
				case MSG_CONFIG_GET_STRING:
				case MSG_CONFIG_GET_VALUE:
					try {
						Configuration::ValueIterator *i = config_->get_value(path);
						if (i->next()) {
							send_value(msg->clid(), i);
						} else {
							send_inv_value(msg->clid(), path);
						}
						delete i;
					} catch (ConfigurationException &e) {
						LibLogger::log_warn("ConfigNetworkHandler",
						                    "get value: Value %s could not be found",
						                    path);
						LibLogger::log_warn("ConfigNetworkHandler", e);
					}
					break;
				}
			}
		} else if ((msg->msgid() >= MSG_CONFIG_SET_BEGIN) && (msg->msgid() <= MSG_CONFIG_SET_END)) {
			bool success = false;

			char path[CONFIG_MSG_PATH_LENGTH + 1];
			if (msg->payload_size() < sizeof(config_descriptor_t)) {
				LibLogger::log_warn("ConfigNetworkHandler",
				                    "inbound set: payload is too small"
				                    "(%zu is less than %zu bytes",
				                    msg->payload_size(),
				                    sizeof(config_descriptor_t));
				send_inv_value(msg->clid(), "?");
			} else {
				config_descriptor_t *cd      = (config_descriptor_t *)msg->payload();
				path[CONFIG_MSG_PATH_LENGTH] = 0;
				strncpy(path, cd->path, CONFIG_MSG_PATH_LENGTH);

				switch (msg->msgid()) {
				case MSG_CONFIG_SET_FLOAT:
				case MSG_CONFIG_SET_DEFAULT_FLOAT:
					try {
						float *vs = (float *)((char *)msg->payload() + sizeof(config_descriptor_t));
						if (cd->num_values > 0) {
							std::vector<float> values(cd->num_values);
							for (unsigned int i = 0; i < cd->num_values; ++i) {
								values[i] = vs[i];
							}
							config_->set_floats(path, values);
						} else {
							if (msg->msgid() == MSG_CONFIG_SET_FLOAT) {
								config_->set_float(path, *vs);
							} else {
								config_->set_default_float(path, *vs);
							}
						}
						success = true;
					} catch (Exception &e) {
						send_inv_value(msg->clid(), path);
						LibLogger::log_warn("ConfigNetworkHandler",
						                    "set float: Value %s could not be set",
						                    path);
						LibLogger::log_warn("ConfigNetworkHandler", e);
					}
					break;

				case MSG_CONFIG_SET_UINT:
				case MSG_CONFIG_SET_DEFAULT_UINT:
					try {
						uint32_t *vs = (uint32_t *)((char *)msg->payload() + sizeof(config_descriptor_t));
						if (cd->num_values > 0) {
							std::vector<unsigned int> values(cd->num_values);
							for (unsigned int i = 0; i < cd->num_values; ++i) {
								values[i] = vs[i];
							}
							config_->set_uints(path, values);
						} else {
							if (msg->msgid() == MSG_CONFIG_SET_UINT) {
								config_->set_uint(path, *vs);
							} else {
								config_->set_default_uint(path, *vs);
							}
						}
						success = true;
					} catch (Exception &e) {
						send_inv_value(msg->clid(), path);
						LibLogger::log_warn("ConfigNetworkHandler",
						                    "set uint: Value %s could not be set",
						                    path);
						LibLogger::log_warn("ConfigNetworkHandler", e);
					}
					break;

				case MSG_CONFIG_SET_INT:
				case MSG_CONFIG_SET_DEFAULT_INT:
					try {
						int32_t *vs = (int32_t *)((char *)msg->payload() + sizeof(config_descriptor_t));
						if (cd->num_values > 0) {
							std::vector<int> values(cd->num_values);
							for (unsigned int i = 0; i < cd->num_values; ++i) {
								values[i] = vs[i];
							}
							config_->set_ints(path, values);
						} else {
							if (msg->msgid() == MSG_CONFIG_SET_INT) {
								config_->set_int(path, *vs);
							} else {
								config_->set_default_int(path, *vs);
							}
						}
						success = true;
					} catch (Exception &e) {
						send_inv_value(msg->clid(), path);
						LibLogger::log_warn("ConfigNetworkHandler", "set int: Value %s could not be set", path);
						LibLogger::log_warn("ConfigNetworkHandler", e);
					}
					break;

				case MSG_CONFIG_SET_BOOL:
				case MSG_CONFIG_SET_DEFAULT_BOOL:
					try {
						int32_t *vs = (int32_t *)((char *)msg->payload() + sizeof(config_descriptor_t));
						if (cd->num_values > 0) {
							std::vector<bool> values(cd->num_values);
							for (unsigned int i = 0; i < cd->num_values; ++i) {
								values[i] = (vs[i] != 0);
							}
							config_->set_bools(path, values);
						} else {
							if (msg->msgid() == MSG_CONFIG_SET_INT) {
								config_->set_bool(path, (*vs != 0));
							} else {
								config_->set_default_bool(path, (*vs != 0));
							}
						}
						success = true;
					} catch (Exception &e) {
						send_inv_value(msg->clid(), path);
						LibLogger::log_warn("ConfigNetworkHandler",
						                    "set bool: Value %s could not be set",
						                    path);
						LibLogger::log_warn("ConfigNetworkHandler", e);
					}
					break;

				case MSG_CONFIG_SET_STRING:
				case MSG_CONFIG_SET_DEFAULT_STRING:
					try {
						char *tmp = ((char *)msg->payload() + sizeof(config_descriptor_t));

						if (cd->num_values > 0) {
							std::vector<std::string> values(cd->num_values);
							for (unsigned int i = 0; i < cd->num_values; ++i) {
								config_string_value_t *sv         = (config_string_value_t *)tmp;
								char *                 msg_string = tmp + sizeof(config_string_value_t);
								tmp += sizeof(config_string_value_t) + sv->s_length + 1;
								values[i] = std::string(msg_string, sv->s_length);
							}
							config_->set_strings(path, values);
						} else {
							config_string_value_t *sv         = (config_string_value_t *)tmp;
							char *                 msg_string = tmp + sizeof(config_string_value_t);
							std::string            value      = std::string(msg_string, sv->s_length);
							if (msg->msgid() == MSG_CONFIG_SET_INT) {
								config_->set_string(path, value);
							} else {
								config_->set_default_string(path, value);
							}
						}
						success = true;
					} catch (Exception &e) {
						send_inv_value(msg->clid(), path);
						LibLogger::log_warn("ConfigNetworkHandler",
						                    "set string: Value %s could not be set",
						                    path);
						LibLogger::log_warn("ConfigNetworkHandler", e);
					}
					break;
				}
			}

			if (success) {
				try {
					Configuration::ValueIterator *i = config_->get_value(path);
					if (i->next()) {
						send_value(msg->clid(), i);
					} else {
						send_inv_value(msg->clid(), path);
					}
					delete i;
				} catch (ConfigurationException &e) {
					LibLogger::log_warn("ConfigNetworkHandler",
					                    "get value: Value %s could not be found",
					                    path);
					LibLogger::log_warn("ConfigNetworkHandler", e);
				}
			}
		}

		msg->unref();
		inbound_queue_.pop_locked();
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
	inbound_queue_.push_locked(msg);
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
	subscribers_.lock();
	if (find(subscribers_.begin(), subscribers_.end(), clid) != subscribers_.end()) {
		LibLogger::log_warn(
		  "ConfigNetworkHandler",
		  "Client %u disconnected without closing the config, removing from list of subscribers",
		  clid);
		subscribers_.remove(clid);
	}
	subscribers_.unlock();
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

	subscribers_.lock();
	for (sit_ = subscribers_.begin(); sit_ != subscribers_.end(); ++sit_) {
		try {
			send_value(*sit_, v);
		} catch (Exception &e) {
			LibLogger::log_warn("ConfigNetworkHandler",
			                    "config_value_changed: Value for %s could not be sent "
			                    "to client %u",
			                    path,
			                    *sit_);
			LibLogger::log_warn("ConfigNetworkHandler", e);
		}
	}
	subscribers_.unlock();
}

void
ConfigNetworkHandler::config_comment_changed(const Configuration::ValueIterator *v)
{
}

void
ConfigNetworkHandler::config_value_erased(const char *path)
{
	subscribers_.lock();
	for (sit_ = subscribers_.begin(); sit_ != subscribers_.end(); ++sit_) {
		try {
			config_value_erased_msg_t *r = prepare_msg<config_value_erased_msg_t>(path, false);
			hub_->send(*sit_,
			           FAWKES_CID_CONFIGMANAGER,
			           MSG_CONFIG_VALUE_ERASED,
			           r,
			           sizeof(config_value_erased_msg_t));
		} catch (Exception &e) {
			LibLogger::log_warn("ConfigNetworkHandler",
			                    "configValueErased: Value for %s could not be sent "
			                    "to client %u",
			                    path,
			                    *sit_);
		}
	}
	subscribers_.unlock();
}

} // end namespace fawkes
