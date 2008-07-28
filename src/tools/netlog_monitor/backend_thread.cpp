
/***************************************************************************
 *  backend_thread.h - Fawkes Network Log Monitor Backend Thread
 *
 *  Created: Sun Dec 09 20:56:12 2007
 *  Copyright  2007  Daniel Beck
 *
 *  $Id$
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

#include <tools/netlog_monitor/backend_thread.h>
#include <tools/netlog_monitor/netlog_monitor.h>

#include <netcomm/fawkes/client.h>
#include <netcomm/fawkes/component_ids.h>
#include <netcomm/dns-sd/avahi_thread.h>
#include <netcomm/utils/network_logger.h>

#include <core/threading/wait_condition.h>

#include <cstring>

using namespace fawkes;

/** @class NetLogMonitorBackendThread backend_thread.h <tools/netlog_monitor/backend_thread.h>
 * Backend for the netlog monitor.
 *
 * @author Daniel Beck
 */

/** Constructor.
 * @param frontend pointer to the frontend
 */
NetLogMonitorBackendThread::NetLogMonitorBackendThread(NetLogMonitor* frontend)
  : Thread("NetLogMonitorBackendThread")
{
  m_frontend = frontend;

  m_clients.clear();
  m_new_clients.clear();
  m_disconnected_clients.clear();
  m_client_id = 0;
  
  
  m_avahi = new AvahiThread();
  m_avahi->watch_service("_fawkes._tcp", this);
  m_avahi->start();

  m_sleep = new WaitCondition();
}

/** Destructor. */
NetLogMonitorBackendThread::~NetLogMonitorBackendThread()
{
  if ( connected() )
    { disconnect(); }
  
  m_avahi->cancel();
  m_avahi->join();
  delete m_avahi;

  ClientMap::iterator cit;
  m_new_clients.lock();
  for (cit = m_new_clients.begin(); cit != m_new_clients.end(); ++cit)
    {
      delete cit->second;
    }
  m_new_clients.clear();
  m_new_clients.unlock();

  delete m_sleep;
}

/** Connect method.
 * @param host the host
 * @param port the port
 */
void
NetLogMonitorBackendThread::connect(const char* host, unsigned short int port)
{
  unsigned int id = m_client_id++;

  FawkesNetworkClient* c = new FawkesNetworkClient(id, host, port);

  m_new_clients.lock();
  m_new_clients[id] = c;
  m_new_clients.unlock();

  m_host_names.lock();
  m_host_names[id] = std::string(host);
  m_host_names.unlock();

  m_sleep->wake_all();
}

/** Disconnect. */
void
NetLogMonitorBackendThread::disconnect()
{
  m_clients.lock();
  size_t n = m_clients.size();
  m_clients.unlock();

  printf(" ** disconnect(): %zu clients active\n", n);

  ClientMap::iterator cit;
  for (cit = m_clients.begin(); cit != m_clients.end(); ++cit)
    {
      FawkesNetworkClient* c = cit->second;
      c->disconnect();
      c->deregister_handler(FAWKES_CID_NETWORKLOGGER);
      delete c;
    }
  m_clients.lock();
  m_clients.clear();
  m_clients.unlock();

  m_host_names.lock();
  m_host_names.clear();
  m_host_names.unlock();

  m_hosts.lock();
  m_hosts.clear();
  m_hosts.unlock();
}

/** Returns connection status.
 * @return true if connected, false otherwise
 */
bool
NetLogMonitorBackendThread::connected() const
{
  return (m_clients.size() != 0);
}

void
NetLogMonitorBackendThread::loop()
{
  size_t num_new_clients;
  size_t num_disconnected_clients;

  m_disconnected_clients.lock();
  num_disconnected_clients = m_disconnected_clients.size();
  m_disconnected_clients.unlock();

  m_new_clients.lock();
  num_new_clients = m_new_clients.size();
  m_new_clients.unlock();

  if ( num_new_clients == 0 && 
       num_disconnected_clients == 0 )
    // nothing to do
    { m_sleep->wait(); }
  
  m_new_clients.lock();
  num_new_clients = m_new_clients.size();
  m_new_clients.unlock();

  // handle new connections
  ClientMap::iterator ncit;
  if ( num_new_clients != 0 )
    {
      m_new_clients.lock();
      ncit = m_new_clients.begin();
      m_new_clients.unlock();

      FawkesNetworkClient* c = ncit->second;
      unsigned int id = c->id();
      c->register_handler(this, FAWKES_CID_NETWORKLOGGER);
      try
	{
	  c->connect();
	  FawkesNetworkMessage* m = new FawkesNetworkMessage(FAWKES_CID_NETWORKLOGGER,
							     NetworkLogger::MSGTYPE_SUBSCRIBE);
	  c->enqueue(m);
	  m->unref();
	}
      catch (Exception& e)
	{
	  e.print_trace();
	  
	  m_host_names.lock();
	  m_host_names.erase(id);
	  m_host_names.unlock();

	  delete c;
	}

      m_new_clients.lock();
      m_new_clients.erase(id);
      m_new_clients.unlock();
    }

  // delete disconnected clients
  m_disconnected_clients.lock();
  if (m_disconnected_clients.size() != 0)
    {
      delete m_disconnected_clients.front();
      m_disconnected_clients.pop();
    }
  m_disconnected_clients.unlock();

}

void
NetLogMonitorBackendThread::deregistered(unsigned int id) throw()
{
}

void
NetLogMonitorBackendThread::connection_established(unsigned int id) throw()
{
  if ( m_new_clients.find(id) == m_new_clients.end() )
    { return; }
  
  if ( m_clients.find(id) != m_clients.end() )
    { return; }

  if ( m_new_clients[id]->id() != id )
    { 
      throw Exception("Client ID mismatch: client's id=%d param=%d", 
		      m_new_clients[id]->id(), id);
    }
  
  m_clients.lock();
  m_clients[id] = m_new_clients[id];
  m_clients.unlock();

  m_frontend->signal_new_host(id, m_host_names[id]);
}

void
NetLogMonitorBackendThread::connection_died(unsigned int id) throw()
{
  if ( m_clients.find(id) == m_clients.end() )
    { throw Exception("Cliend ID not registered"); }

  if ( m_host_names.find(id) == m_host_names.end() )
    { throw Exception("No host name is registered for this cliend ID"); }

  m_disconnected_clients.push_locked( m_clients[id] );

  m_clients.lock();
  m_clients.erase(id);
  m_clients.unlock();

  m_hosts.lock();
  m_hosts.erase( m_host_names[id] );
  m_hosts.unlock();
  
  m_host_names.lock();
  m_host_names.erase(id);
  m_host_names.unlock();

  m_sleep->wake_all();

  m_frontend->signal_conn_status_chg(id, false);
}

void
NetLogMonitorBackendThread::inbound_received(FawkesNetworkMessage* msg,
					     unsigned int id) throw()
{
  if (msg->cid() != FAWKES_CID_NETWORKLOGGER)
    {
      return;
    }
  if ( (msg->cid() == FAWKES_CID_NETWORKLOGGER) &&
       (msg->msgid() == NetworkLogger::MSGTYPE_LOGMESSAGE) )
    {
      NetworkLoggerMessageContent* content = msg->msgc<NetworkLoggerMessageContent>();

      char* loglevel;
      char* time;

      switch ( content->get_loglevel() )
	{
	case Logger::LL_DEBUG:
	  asprintf(&loglevel, "DBG");
	  break;
	case Logger::LL_INFO:
	  asprintf(&loglevel, "INFO");
	  break;
	case Logger::LL_WARN:
	  asprintf(&loglevel, "WRN");
	  break;
	case Logger::LL_ERROR:
	  asprintf(&loglevel, "ERR");
	  break;
	default:
	  break;
	}

      struct timeval t = content->get_time();
      struct tm time_tm;
      localtime_r(&(t.tv_sec), &time_tm);
      asprintf(&time, "%02d:%02d:%02d.%06ld", time_tm.tm_hour,
	       time_tm.tm_min, time_tm.tm_sec, t.tv_usec);
      
      //       printf(" ** inbound_received(): received new log message from client %d:\n", id);
      //       printf("    loglevel: %s  time: %s  component: %s\n", loglevel, time, content->get_component());
      //       printf("    message: %s\n", content->get_message());
      
      
      m_frontend->signal_new_message( id, loglevel, time, 
				      content->get_component(),
				      content->get_message() );
    }
}

void
NetLogMonitorBackendThread::all_for_now()
{
}

void
NetLogMonitorBackendThread::cache_exhausted()
{
}

void
NetLogMonitorBackendThread::browse_failed( const char* name,
					   const char* type,
					   const char* domain )
{
}

void
NetLogMonitorBackendThread::service_added( const char* name,
					   const char* type,
					   const char* domain,
					   const char* host_name,
					   const struct sockaddr* addr,
					   const socklen_t addr_size,
					   uint16_t port,
					   std::list<std::string>& txt,
					   int flags )
{
  unsigned int id;

  m_hosts.lock();
  if ( m_hosts.find(host_name) != m_hosts.end() )
    {
      m_hosts.unlock();
      return;
    }
  else
    {
      id = m_client_id++;
      m_hosts[host_name] = id;
      m_hosts.unlock();
    }

  m_new_clients.lock();
  m_new_clients[id] = new FawkesNetworkClient(id, host_name, port);
  m_new_clients.unlock();
  m_host_names.lock();
  m_host_names[id] = std::string(host_name);
  m_host_names.unlock();

  m_sleep->wake_all();
}

void
NetLogMonitorBackendThread::service_removed( const char* name,
					     const char* type,
					     const char* domain )
{
}
