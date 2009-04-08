
/***************************************************************************
 *  worldinfo_viewer.h - World Info Viewer
 *
 *  Created: Wed April 09 20:09:01 2008
 *  Copyright  2008  Daniel Beck
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

#ifndef __TOOLS_WORLDINFO_VIEWER_WORLDINFO_VIEWER_H_
#define __TOOLS_WORLDINFO_VIEWER_WORLDINFO_VIEWER_H_

#include <gtkmm.h>
#include <libglademm/xml.h>
#include <cairomm/context.h>

class FieldView;
namespace fawkes {
  class WorldInfoDataContainer;
  class BatteryInterface;
}

class WorldInfoViewer : public Gtk::Window
{
 public:
  WorldInfoViewer( Glib::RefPtr<Gnome::Glade::Xml> ref_xml,
		   fawkes::WorldInfoDataContainer* data_container );
  virtual ~WorldInfoViewer();
  
  Gtk::Window& get_window() const;
  
  bool update();
  void gamestate_changed();
  
 private:
  class RobotRecord : public Gtk::TreeModelColumnRecord
  {
   public:
    RobotRecord()
    {
      add(hostname);
      add(fqdn);
      add(show_pose);
      add(show_ball);
      add(show_opponents);
    }
    
    Gtk::TreeModelColumn<Glib::ustring> hostname;
    Gtk::TreeModelColumn<Glib::ustring> fqdn;
    Gtk::TreeModelColumn<bool> show_pose;
    Gtk::TreeModelColumn<bool> show_ball;
    Gtk::TreeModelColumn<bool> show_opponents;
  };

  Gtk::Widget* get_widget( Glib::RefPtr<Gnome::Glade::Xml> ref_xml,
			   const char* widget_name ) const;

  // signal handlers
  void on_show_pose_toggled( const Glib::ustring& path );
  void on_show_ball_toggled( const Glib::ustring& path );
  void on_show_opponents_toggled( const Glib::ustring& path );

  Gtk::Window*    m_wnd_main;
  Gtk::VBox*      m_vbx_field;
  Gtk::TreeView*  m_trv_robots;
  Gtk::Statusbar* m_stb_status;

  unsigned int m_stb_message_id;
  
  FieldView* m_field_view;

  RobotRecord m_robot_record;
  Glib::RefPtr<Gtk::ListStore> m_robots_list;

  fawkes::WorldInfoDataContainer* m_data_container;

  unsigned int m_robot_id;
  std::map<Glib::ustring, unsigned int> m_robots;
  std::map<unsigned int, Gtk::TreeModel::Row> m_list_entries;
};

#endif /*  __TOOLS_WORLDINFO_VIEWER_WORLDINFO_VIEWER_H_ */
