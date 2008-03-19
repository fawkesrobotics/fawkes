
/***************************************************************************
 *  backend_thread.h - Backend thread of the Plugin Tool Gui
 *
 *  Created: Wed Nov 08 09:52:45 2007
 *  Copyright  2007  Daniel Beck
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

#ifndef __TOOLS_PLUGIN_PLUGIN_GUI_BACKEND_THREAD_H_
#define __TOOLS_PLUGIN_PLUGIN_GUI_BACKEND_THREAD_H_

#include <core/threading/thread.h>
#include <netcomm/service_discovery/browse_handler.h>
#include <netcomm/fawkes/client_handler.h>

#include <map>
#include <vector>
#include <string>

class PluginGui;
class FawkesNetworkClient;
class WaitCondition;
class AvahiThread;

class PluginGuiBackendThread : public Thread, 
  public FawkesNetworkClientHandler,
  public ServiceBrowseHandler
{
 public:
  PluginGuiBackendThread(PluginGui* gui);
  virtual ~PluginGuiBackendThread();

  bool connect(const char* host, unsigned short int port);
  bool disconnect();
  bool is_connected();

  void loop();

  // client handler
  void deregistered(unsigned int id) throw();
  void connection_died(unsigned int id) throw();
  void connection_established(unsigned int id) throw();
  void inbound_received(FawkesNetworkMessage* m,
			unsigned int id) throw();

  // service browser handler
  void all_for_now();
  void cache_exhausted();
  void browse_failed( const char* name,
		      const char* type,
		      const char* domain );
  void service_added( const char* name,
		      const char* type,
		      const char* domain,
		      const char* host_name,
		      const struct sockaddr* addr,
		      const socklen_t addr_size,
		      uint16_t port,
		      std::list<std::string>& txt,
		      int flags );
  void service_removed( const char* name,
			const char* type,
			const char* domain );

  std::map<std::string,bool>& plugin_status();
  std::vector<std::string> hosts();
  
  void request_load(const char* plugin_name);
  void request_unload(const char* plugin_name);

 private:
  FawkesNetworkClient* m_client;
  AvahiThread* m_avahi;
  std::map<std::string,bool> m_plugin_status;
  std::map<std::string,std::string> m_hosts;
  PluginGui* m_gui;
  bool m_connected;
  bool m_connection_died;

  WaitCondition* m_longsleep;
};

#endif /* __TOOLS_PLUGIN_PLUGIN_GUI_BACKEND_THREAD_H_ */
