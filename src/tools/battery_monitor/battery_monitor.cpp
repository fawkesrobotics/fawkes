
/***************************************************************************
 *  battery_monitor.cpp - Fawkes Battery Monitor
 *
 *  Created: Mon Apr 06 17:11:55 2009
 *  Copyright  2009  Daniel Beck
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

#include "battery_monitor.h"
#include "battery_monitor_treeview.h"

#include <netcomm/dns-sd/avahi_thread.h>

using namespace std;
using namespace fawkes;

/** @class BatteryMonitor tools/battery_monitor/battery_monitor.h
 * A battery monitor.
 * @author Daniel Beck
 */

/** Constructor.
 * @param builder builder to get widgets from
 */
BatteryMonitor::BatteryMonitor(Glib::RefPtr<Gtk::Builder> builder)
{
  builder->get_widget("wndMain", m_wnd_main);
  m_trv_battery = NULL;
  builder->get_widget_derived( "trvBattery", m_trv_battery );
  builder->get_widget("btnQuit", m_btn_quit);
  m_btn_quit->signal_clicked().connect( sigc::mem_fun( *this, &BatteryMonitor::on_btn_quit_clicked ) );

  m_avahi = new AvahiThread();
  m_avahi->watch_service( "_fawkes._tcp", this );
  m_avahi->start();
}

/** Destructor */
BatteryMonitor::~BatteryMonitor()
{
  m_avahi->cancel();
  m_avahi->join();
  delete m_avahi;
}

/** Obtain the main window.
 * @return the main window
 */
Gtk::Window&
BatteryMonitor::get_window() const
{
  return *m_wnd_main;
}

void
BatteryMonitor::all_for_now()
{
}

void
BatteryMonitor::cache_exhausted()
{
}

void
BatteryMonitor::browse_failed( const char* name,
			       const char* type,
			       const char* domain )
{
}

void
BatteryMonitor::service_added( const char* name,
			       const char* type,
			       const char* domain,
			       const char* host_name,
			       const char* interface,
			       const struct sockaddr* addr,
			       const socklen_t addr_size,
			       uint16_t port,
			       std::list<std::string>& txt,
			       int flags )
{
  string host( host_name );
  string service( name );
  m_services[ service ] = host_name;
  m_trv_battery->add_host( host_name );
}

void
BatteryMonitor::service_removed( const char* name,
				 const char* type,
				 const char* domain )
{
  std::map< string, string >::iterator i =  m_services.find( string( name ) );
  if ( i != m_services.end() )
  { m_trv_battery->rem_host( (i->second).c_str() ); }
}

void
BatteryMonitor::on_btn_quit_clicked()
{
  m_wnd_main->hide();
}
