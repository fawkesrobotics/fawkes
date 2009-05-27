
/***************************************************************************
 *  net_handler.h - Fawkes configuration network handler
 *
 *  Created: Sat Jan 06 22:53:23 2007
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __CONFIG_NET_HANDLER_H_
#define __CONFIG_NET_HANDLER_H_

#include <core/threading/thread.h>
#include <netcomm/fawkes/handler.h>
#include <core/utils/lock_queue.h>
#include <core/utils/lock_list.h>

#include <config/net_messages.h>
#include <config/config.h>

#include <cstdlib>
#include <map>
#include <string>
#include <utility>

namespace fawkes {

class FawkesNetworkHub;


class ConfigNetworkHandler
: public Thread,
  public FawkesNetworkHandler,
  public ConfigurationChangeHandler
{
 public:
  ConfigNetworkHandler(Configuration *config, FawkesNetworkHub *hub);
  ~ConfigNetworkHandler();

  /* from FawkesNetworkHandler interface */
  virtual void handle_network_message(FawkesNetworkMessage *msg);
  virtual void client_connected(unsigned int clid);
  virtual void client_disconnected(unsigned int clid);
  virtual void loop();

  /* from ConfigurationChangeHandler interface */
  virtual void config_tag_changed(const char *new_location);
  virtual void config_value_changed(const char *path, int value);
  virtual void config_value_changed(const char *path, unsigned int value);
  virtual void config_value_changed(const char *path, float value);
  virtual void config_value_changed(const char *path, bool value);
  virtual void config_value_changed(const char *path, const char *value);
  virtual void config_value_erased(const char *path);

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void send_value(unsigned int clid, Configuration::ValueIterator *i);
  void send_inv_value(unsigned int clid, const char *path);

  template <typename T>
    T *  prepare_msg(const char *path, bool is_default)
    {
      T * m = (T *)calloc(1, sizeof(T));
      strncpy(m->cp.path, path, CONFIG_MSG_PATH_LENGTH);
      m->cp.is_default = is_default;
      return m;
    }

  Configuration                       *__config;
  FawkesNetworkHub                    *__hub;
  LockQueue< FawkesNetworkMessage * >  __inbound_queue;

  LockList< unsigned int >             __subscribers;
  LockList< unsigned int >::iterator   __sit;
};

} // end namespace fawkes

#endif
