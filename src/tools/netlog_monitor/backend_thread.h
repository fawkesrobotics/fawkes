
/***************************************************************************
 *  backend_thread.h - Fawkes Network Log Monitor Backend Thread
 *
 *  Created: Sun Dec 09 20:38:21 2007
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

#ifndef __TOOLS_NETLOG_MONITOR_BACKEND_THREAD_H_
#define __TOOLS_NETLOG_MONITOR_BACKEND_THREAD_H_

#include <core/threading/thread.h>
#include <core/utils/lock_queue.h>
#include <core/utils/lock_map.h>
#include <netcomm/fawkes/client_handler.h>
#include <netcomm/service_discovery/browse_handler.h>

class NetLogMonitor;
class FawkesNetworkClient;
class WaitCondition;
class AvahiThread;

class NetLogMonitorBackendThread : public Thread,
  public FawkesNetworkClientHandler,
  public ServiceBrowseHandler
{
 public:
  NetLogMonitorBackendThread(NetLogMonitor* nlm);
  virtual ~NetLogMonitorBackendThread();

  void connect(const char* host, unsigned short int port = 1910);
  void disconnect();

  bool connected() const;

  // thread
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

 private:
  typedef LockMap<unsigned int, FawkesNetworkClient*> ClientMap;
  ClientMap m_new_clients;
  ClientMap m_clients;
  unsigned int m_client_id;

  LockMap<std::string, unsigned int> m_hosts;
  LockMap<unsigned int, std::string> m_host_names;

  
  LockQueue<FawkesNetworkClient*> m_disconnected_clients;

  NetLogMonitor* m_frontend;
  WaitCondition* m_sleep;
  AvahiThread* m_avahi;
};

#endif /* __TOOLS_NETLOG_MONITOR_BACKEND_THREAD_H_ */
