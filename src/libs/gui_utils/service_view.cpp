
/***************************************************************************
 *  service_model.cpp - Manages list of discovered services of given type
 *
 *  Created: Mon Sep 29 16:37:14 2008
 *  Copyright  2008  Daniel Beck
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

#include <gui_utils/service_view.h>
#include <netcomm/dns-sd/avahi_thread.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

using namespace std;
using namespace fawkes;

/** @class fawkes::ServiceView::ServiceRecord gui_utils/service_view.h
 * Detects services and manages information about detected services.
 *
 * @author Daniel Beck
 */

/** @class fawkes::ServiceView gui_utils/service_view.h 
 * Abstract base class for widgets that allow to view the detected
 * services of a certain type.
 *
 * @author Daniel Beck
 */

/** @fn void fawkes::ServiceView::initialize()
 * In derived classes use this function to initialize the GUI
 * componentes, e.g. set the model for a tree view.
 */

/** @var fawkes::ServiceView::m_service_list
 * Storage object.
 */

/** @var fawkes::ServiceView::m_service_record
 * Column record class
 */

/** @var fawkes::ServiceView::m_avahi
 * Avahi thread.
 */

/** @var fawkes::ServiceView::m_signal_service_added
 * This signal is emitted whenever a new service has been added.
 */

/** @var fawkes::ServiceView::m_signal_service_removed
 * This signal is emitted whenever a service is removed
 */

/** @struct fawkes::ServiceView::ServiceAddedRecord
 * Data structure to hold information about a newly added services.
 */ 

/** @struct fawkes::ServiceView::ServiceRemovedRecord
 * Data structure to hold information about a recently removed services.
 */ 

/** @var fawkes::ServiceView::m_added_services
 * Queue that holds the newly added services.
 */

/** @var fawkes::ServiceView::m_removed_services
 * Queue that holds the recently removed services.
 */

/** Constructor.
 * @param service the service identifier
 */
ServiceView::ServiceView(const char* service)
{
  m_avahi = new AvahiThread();
  m_avahi->watch_service(service, this);
  m_avahi->start();

  m_own_avahi_thread = true;

  m_service_list = Gtk::ListStore::create(m_service_record);

  m_signal_service_added.connect( sigc::mem_fun(*this, &ServiceView::on_service_added) );
  m_signal_service_removed.connect( sigc::mem_fun(*this, &ServiceView::on_service_removed) );
}

/** Constructor.
 * @param avahi_thread an AvahiThread that already watches for the
 * desired type of services
 */
ServiceView::ServiceView(fawkes::AvahiThread* avahi_thread)
{
  m_avahi = avahi_thread;
  m_own_avahi_thread = false;

  m_service_list = Gtk::ListStore::create(m_service_record);
}

/** Destructor. */
ServiceView::~ServiceView()
{
  if (m_own_avahi_thread)
    { 
      m_avahi->cancel();
      m_avahi->join();
      delete m_avahi;
    }
}

void
ServiceView::all_for_now()
{
}

void
ServiceView::cache_exhausted()
{
}

void
ServiceView::browse_failed( const char* name,
			    const char* type,
			    const char* domain )
{
}

void
ServiceView::service_added( const char* name,
			    const char* type,
			    const char* domain,
			    const char* host_name,
			    const struct sockaddr* addr,
			    const socklen_t addr_size,
			    uint16_t port,
			    std::list<std::string>& txt,
			    int flags )
{
  ServiceAddedRecord s;
  char ipaddr[INET_ADDRSTRLEN];
  struct sockaddr_in *saddr = (struct sockaddr_in *)addr;
  s.name = string(name);
  s.type = string(type);
  s.domain = string(domain);
  s.hostname = string(host_name);
  s.ipaddr = inet_ntop(AF_INET, &(saddr->sin_addr), ipaddr, sizeof(ipaddr));
  s.port = port;

  m_added_services.push_locked(s);

  m_signal_service_added();
}

void
ServiceView::service_removed( const char* name,
			      const char* type,
			      const char* domain )
{
  ServiceRemovedRecord s;
  s.name = string(name);
  s.type = string(type);
  s.domain = string(domain);

  m_removed_services.push_locked(s);

  m_signal_service_removed();
}

/** Signal handler for the service-added signal. */
void
ServiceView::on_service_added()
{
  m_added_services.lock();

  while ( !m_added_services.empty() )
    {
      ServiceAddedRecord& s  = m_added_services.front();

      Gtk::TreeModel::Row row = *m_service_list->append();

      row[m_service_record.name]     = s.name;
      row[m_service_record.type]     = s.type;
      row[m_service_record.domain]   = s.domain;
      row[m_service_record.hostname] = s.hostname;
      row[m_service_record.ipaddr]   = s.ipaddr;
      row[m_service_record.port]     = s.port;

      m_added_services.pop();
    }

  m_added_services.unlock();
}

/** Signal handler for the service-removed signal. */
void
ServiceView::on_service_removed()
{
  m_removed_services.lock();

  while ( !m_removed_services.empty() )
    {
      ServiceRemovedRecord& s = m_removed_services.front();

      Gtk::TreeIter iter;
      iter = m_service_list->children().begin();
      
      while ( iter != m_service_list->children().end() )
	{
	  Gtk::TreeModel::Row row = *iter;
	  if ( (row[m_service_record.name]   == s.name) &&
	       (row[m_service_record.type]   == s.type) &&
	       (row[m_service_record.domain] == s.domain) )
	    {
	      iter = m_service_list->erase(iter);
	      m_service_list->row_deleted( m_service_list->get_path(iter) );
	    }
	  else
	    { ++iter; }
	}

      m_removed_services.pop();
    }

  m_removed_services.unlock();
}
