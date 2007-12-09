
/***************************************************************************
 *  config_manager.h - Fawkes configuration manager
 *
 *  Created: Sat Jan 06 22:53:23 2007
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

#ifndef __FAWKES_CONFIG_MANAGER_H_
#define __FAWKES_CONFIG_MANAGER_H_

#include <netcomm/fawkes/handler.h>
#include <core/utils/lock_queue.h>
#include <core/utils/lock_list.h>

#include <config/net_messages.h>
#include <config/config.h>

#include <stdlib.h>
#include <map>
#include <string>
#include <utility>

class ThreadManager;
class FawkesNetworkHub;
class Mutex;

class FawkesConfigManager : public FawkesNetworkHandler, public ConfigurationChangeHandler
{
 public:
  FawkesConfigManager(Configuration *config);
  ~FawkesConfigManager();

  void set_hub(FawkesNetworkHub *hub);

  /* from FawkesNetworkHandler interface */
  virtual void handle_network_message(FawkesNetworkMessage *msg);
  virtual void client_connected(unsigned int clid);
  virtual void client_disconnected(unsigned int clid);
  virtual void process_after_loop();

  /* from ConfigurationChangeHandler interface */
  virtual void config_tag_changed(const char *new_location);
  virtual void config_value_changed(const char *path, int value);
  virtual void config_value_changed(const char *path, unsigned int value);
  virtual void config_value_changed(const char *path, float value);
  virtual void config_value_changed(const char *path, bool value);
  virtual void config_value_changed(const char *path, const char *value);
  virtual void config_value_erased(const char *path);

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

  Configuration  *config;
  Mutex *mutex;
  LockQueue< FawkesNetworkMessage * > inbound_queue;

  LockList< unsigned int >           subscribers;
  LockList< unsigned int >::iterator sit;

  FawkesNetworkHub *hub;
};

#endif
