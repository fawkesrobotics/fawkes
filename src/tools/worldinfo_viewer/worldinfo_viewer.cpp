
/***************************************************************************
 *  worldinfo_viewer.cpp -  World Info Viewer
 *
 *  Created: Wed April 09 20:13:08 2008
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

#include <tools/worldinfo_viewer/worldinfo_viewer.h>
#include <tools/worldinfo_viewer/field_view.h>
#include <worldinfo_utils/data_container.h>
#include <blackboard/remote.h>
#include <interfaces/battery.h>
#include <vector>
#include <map>
#include <string>
#include <cstdio>

using namespace std;

/** @class WorldInfoViewer <tools/worldinfo_viewer/worldinfo_viewer.h>
 * Main class of the WorldInfoViewer application.
 * @author Daniel Beck
 */

/** Constructor.
 * @param ref_xml reference to the Glade XML file
 * @param data_container pointer to the central instance of the WorldInfoDataContainer
 */
WorldInfoViewer::WorldInfoViewer( Glib::RefPtr<Gnome::Glade::Xml> ref_xml,
				  WorldInfoDataContainer* data_container )
{
  m_wnd_main   = dynamic_cast<Gtk::Window*>( get_widget(ref_xml, "wndMain") );
  m_vbx_field  = dynamic_cast<Gtk::VBox*>( get_widget(ref_xml, "vbxField") );
  m_trv_robots = dynamic_cast<Gtk::TreeView*>( get_widget(ref_xml, "trvRobots") );
  m_stb_status = dynamic_cast<Gtk::Statusbar*>( get_widget(ref_xml, "stbStatus") );

  m_field_view = new FieldView();
  m_vbx_field->pack_start(*m_field_view);
  m_field_view->show();

  m_robots_list = Gtk::ListStore::create(m_robot_record);
  m_trv_robots->set_model(m_robots_list);
  m_trv_robots->append_column("Name", m_robot_record.name);
  m_trv_robots->append_column("Voltage", m_robot_record.voltage);
  //  m_trv_robots->append_column("Obstacles", m_robot_record.show_obstacles);

  m_data_container = data_container;

  m_stb_message_id = m_stb_status->push("No game state information available.");
}

/** Destructor. */
WorldInfoViewer::~WorldInfoViewer()
{
  std::map<Glib::ustring, unsigned int>::iterator rit;
  for (rit = m_robots.begin(); rit != m_robots.end(); ++rit)
    {
      unsigned int id = rit->second;
      RemoteBlackBoard* rbb = m_remote_bbs[id];
      BatteryInterface* bi  = m_battery_interfaces[id];
      rbb->close(bi);
      delete m_remote_bbs[id];
    }

  delete m_wnd_main;
}

/** Obtain the main window of the application.
 * @return reference to the main window
 */
Gtk::Window&
WorldInfoViewer::get_window() const
{
  return *m_wnd_main;
}

Gtk::Widget*
WorldInfoViewer::get_widget(Glib::RefPtr<Gnome::Glade::Xml> ref_xml,
			    const char* widget_name) const
{
  Gtk::Widget* widget;
  ref_xml->get_widget(widget_name, widget);
  if ( !widget )
    {
      char* err_str;
      asprintf(&err_str, "Couldn't find widget %s", widget_name);
      throw std::runtime_error(err_str);
      free(err_str);
    }

  return widget;
}

/** Redraw the field. */
void
WorldInfoViewer::redraw_field()
{
  m_field_view->reset();

  std::map<unsigned int, bool>::iterator ait;
  for (ait = m_robot_active.begin(); ait != m_robot_active.end(); ++ait)
    {
      ait->second = false;
    }

  vector<string>::iterator hit;
  vector<string> hosts = m_data_container->get_hosts();

  HomPoint pos;
  HomPolar pos_rel;
  for (hit = hosts.begin(); hit != hosts.end(); ++hit)
    {
      // robot pose
      HomPose pose;
      Matrix pose_cov;
      if ( m_data_container->get_robot_pose( hit->c_str(), pose, pose_cov) )
	{ m_field_view->set_robot_pose( hit->c_str(), pose.x(), pose.y(), pose.yaw()); }

      // global ball positions
      const char* host_name = (*hit).c_str();
      if ( m_data_container->get_ball_pos_global(host_name, pos) )
	{ m_field_view->set_ball_pos(host_name, pos.x(), pos.y()); }
      else 
      // relative ball positions
	if ( m_data_container->get_ball_pos(host_name, pos_rel, pose_cov) )
	  { m_field_view->set_ball_pos(host_name, pos_rel.x(), pos_rel.y()); }

      // opponents
      // TODO
    }

  // update BB interfaces
  Gtk::TreeModel::Children children = m_robots_list->children();
  for ( Gtk::TreeModel::Children::iterator iter = children.begin();
	iter != children.end(); ++iter )
    {
      Gtk::TreeModel::Row row = *iter;
      
      Glib::ustring name    = row[m_robot_record.name];
      if ( m_robots.find(name) != m_robots.end() )
	{
	  unsigned int id       = m_robots[name];
	  BatteryInterface* bi  = m_battery_interfaces[id];
	  
	  if ( bi->has_writer() )
	    {
	      bi->read();
	      row[m_robot_record.voltage] = bi->voltage();
	    }
	  else
	    { 
	      row[m_robot_record.voltage] = 0.0;
	    }
	}
    }

  m_field_view->queue_draw();
}

/** Call this method whenever the game state changes. */
void
WorldInfoViewer::gamestate_changed()
{
  char* status_string;
  asprintf( &status_string, "Team color: %s  Goal color: %s  Mode: %s  Score: %d:%d  Half: %s",
	    m_data_container->get_own_team_color_string().c_str(),
	    m_data_container->get_own_goal_color_string().c_str(),
	    m_data_container->get_game_state_string().c_str(),
	    m_data_container->get_own_score(),
	    m_data_container->get_other_score(),
	    m_data_container->get_half_string().c_str() );

  m_stb_status->remove_message(m_stb_message_id);
  m_stb_message_id = m_stb_status->push( Glib::ustring(status_string) );

  free(status_string);
}

/** Call this method whenever a new robot was detected. */
void
WorldInfoViewer::robot_added()
{
  vector<string>::iterator hit;
  vector<string> hosts = m_data_container->get_hosts();
  
  for (hit = hosts.begin(); hit != hosts.end(); ++hit)
    {
      if ( m_robots.find( *hit ) == m_robots.end() )
	// add remote BB & interfaces
	{
	  m_robots[*hit] = m_robot_id;
	  
	  // add an entry to the list
	  Gtk::TreeModel::Row row = *m_robots_list->append();
	  row[m_robot_record.name]    = Glib::ustring( (*hit).c_str() );
	  row[m_robot_record.voltage] = 0.0;
	  m_list_entries[m_robot_id] = row;
	  
	  // open new remote blackboard
	  RemoteBlackBoard* rbb = new RemoteBlackBoard( (*hit).c_str(), 1910 );
	  m_remote_bbs[m_robot_id] = rbb;
	  
	  // create new battery interface
	  BatteryInterface* bi = rbb->open_for_reading<BatteryInterface>("Battery");
	  m_battery_interfaces[m_robot_id] = bi;
	  
	  ++m_robot_id;
	}
    }
}

/** Call this method whenever a robot disappeared. */
void
WorldInfoViewer::robot_removed()
{ 
  vector<string>::iterator hit;
  vector<string> hosts = m_data_container->get_hosts();

  std::map<Glib::ustring, unsigned int> cur_robots(m_robots);
  m_robots.clear();

  for (hit = hosts.begin(); hit != hosts.end(); ++hit)
    {
      unsigned int id = cur_robots[*hit];
      m_robots[*hit] = id;
    }
}
