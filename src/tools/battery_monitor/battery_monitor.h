
/***************************************************************************
 *  battery_monitor.h - Fawkes Battery Monitor
 *
 *  Created: Mon Apr 06 17:09:40 2009
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

#ifndef __TOOLS_BATTERY_MONITOR_BATTERY_MONITOR_H_
#define __TOOLS_BATTERY_MONITOR_BATTERY_MONITOR_H_

#include <netcomm/service_discovery/browse_handler.h>

#include <gtkmm.h>

#include <map>
#include <string>

class BatteryMonitorTreeView;

namespace fawkes {
  class AvahiThread;
}

class BatteryMonitor 
: public Gtk::Window,
  fawkes::ServiceBrowseHandler
{
 public:
  BatteryMonitor(Glib::RefPtr<Gtk::Builder> builder);
  ~BatteryMonitor();

  Gtk::Window& get_window() const;

 protected:
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
		      const char* interface,
		      const struct sockaddr* addr,
		      const socklen_t addr_size,
		      uint16_t port,
		      std::list<std::string>& txt,
		      int flags );
  void service_removed( const char* name,
			const char* type,
			const char* domain );

 private:
  void on_btn_quit_clicked();

  Gtk::Window* m_wnd_main;
  BatteryMonitorTreeView* m_trv_battery;
  Gtk::Button* m_btn_quit;

  std::map< std::string, std::string > m_services;
  fawkes::AvahiThread* m_avahi;
};

#endif /*  __TOOLS_BATTERY_MONITOR_BATTERY_MONITOR_H_ */
