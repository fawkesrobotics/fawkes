
/***************************************************************************
 *  battery_monitor_treeview.h - TreeView class for displaying the battery
 *                               status of the robots
 *
 *  Created: Mon Apr 06 15:52:42 2009
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

#ifndef __TOOLS_BATTERY_MONITOR_BATTERY_MONITOR_TREE_VIEW_H_
#define __TOOLS_BATTERY_MONITOR_BATTERY_MONITOR_TREE_VIEW_H_

#include <gtkmm.h>

#include <map>
#include <string>

namespace fawkes {
  class BlackBoard;
  class Interface;
  class BatteryInterface;
  class InterfaceDispatcher;
}

class BatteryMonitorTreeView : public Gtk::TreeView
{
 public:
  BatteryMonitorTreeView(BaseObjectType* cobject,
                         const Glib::RefPtr<Gtk::Builder> &builder);
  virtual ~BatteryMonitorTreeView();

  void add_host( const char* host );
  void rem_host( const char* host );

 protected:
  class BatteryRecord : public Gtk::TreeModelColumnRecord
  {
  public:
    BatteryRecord()
    {
      add( fqdn );
      add( short_name );
      add( absolute_soc );
      add( relative_soc );
      add( current );
      add( voltage );
    }
    
    Gtk::TreeModelColumn< Glib::ustring > fqdn;         /**< The FQDN */
    Gtk::TreeModelColumn< Glib::ustring > short_name;   /**< A shorter hostname (w/o domain) */
    Gtk::TreeModelColumn< float >         absolute_soc; /**< The battery's absolute state of charge */
    Gtk::TreeModelColumn< float >         relative_soc; /**< The battery's relative state of charge */
    Gtk::TreeModelColumn< float >         current;      /**< The battery's current */
    Gtk::TreeModelColumn< float >         voltage;      /**< The battery's voltage */
  };

  BatteryRecord m_battery_record;
  Glib::RefPtr< Gtk::ListStore > m_battery_list;

  std::map< std::string, fawkes::BlackBoard* > m_remote_bbs;
  std::map< std::string, fawkes::BatteryInterface* > m_battery_interfaces;
  std::map< std::string, fawkes::InterfaceDispatcher* > m_interface_dispatcher;

 private:
  void on_data_changed( fawkes::Interface* interface );
  void on_writer_added( fawkes::Interface* interface );
  void on_writer_removed( fawkes::Interface* interface );

  void update();

  Gtk::MessageDialog* m_dlg_warning;

  Glib::Dispatcher m_trigger_update;
  float m_relative_soc_threshold;
  std::map< std::string, unsigned int > m_below_threshold_counter;
};

#endif /* __TOOLS_BATTERY_MONITOR_BATTERY_MONITOR_TREE_VIEW_H_ */
