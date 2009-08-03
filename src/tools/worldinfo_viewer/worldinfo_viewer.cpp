
/***************************************************************************
 *  worldinfo_viewer.cpp -  World Info Viewer
 *
 *  Created: Wed April 09 20:13:08 2008
 *  Copyright  2008  Daniel Beck
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

#include "worldinfo_viewer.h"
#include "field_view.h"

#include <worldinfo_utils/data_container.h>
#include <blackboard/remote.h>
#include <interfaces/BatteryInterface.h>

#include <vector>
#include <map>
#include <string>
#include <cstdio>
#include <cstring>

using namespace std;
using namespace fawkes;


/** @class WorldInfoViewer <tools/worldinfo_viewer/worldinfo_viewer.h>
 * Main class of the WorldInfoViewer application.
 * @author Daniel Beck
 */


/** Constructor.
 * @param ref_xml reference to the Glade XML file
 * @param data_container pointer to the central instance of the
 * WorldInfoDataContainer
 */
WorldInfoViewer::WorldInfoViewer( Glib::RefPtr<Gnome::Glade::Xml> ref_xml,
				  WorldInfoDataContainer* data_container )
{
  m_wnd_main   = dynamic_cast<Gtk::Window*>( get_widget(ref_xml, "wndMain") );
  m_vbx_field  = dynamic_cast<Gtk::VBox*>( get_widget(ref_xml, "vbxField") );
  m_trv_robots = dynamic_cast<Gtk::TreeView*>( get_widget(ref_xml, "trvRobots") );
  m_stb_status = dynamic_cast<Gtk::Statusbar*>( get_widget(ref_xml, "stbStatus") );

  m_field_view = new FieldView( data_container, true, true, false );
  m_vbx_field->pack_start( *m_field_view );
  m_field_view->show();

  m_robots_list = Gtk::ListStore::create( m_robot_record );
  m_trv_robots->set_model( m_robots_list );
  m_trv_robots->append_column( "Name", m_robot_record.hostname );
  m_trv_robots->append_column_editable( "Pose", m_robot_record.show_pose );
  m_trv_robots->append_column_editable( "Ball", m_robot_record.show_ball );
  m_trv_robots->append_column_editable( "Opponents", m_robot_record.show_opponents );

  Gtk::CellRendererToggle* renderer;
  renderer = dynamic_cast< Gtk::CellRendererToggle* >( m_trv_robots->get_column_cell_renderer(1) );
  renderer->signal_toggled().connect( sigc::mem_fun( *this, 
						     &WorldInfoViewer::on_show_pose_toggled ) );
  renderer = dynamic_cast< Gtk::CellRendererToggle* >( m_trv_robots->get_column_cell_renderer(2) );
  renderer->signal_toggled().connect( sigc::mem_fun( *this,
						     &WorldInfoViewer::on_show_ball_toggled ) );
  renderer = dynamic_cast< Gtk::CellRendererToggle* >( m_trv_robots->get_column_cell_renderer(3) );
  renderer->signal_toggled().connect( sigc::mem_fun( *this,
						     &WorldInfoViewer::on_show_opponents_toggled ) );

  m_data_container = data_container;

  m_stb_message_id = m_stb_status->push( "No game state information available." );

  // create timer
  sigc::connection conn = 
    Glib::signal_timeout().connect( sigc::mem_fun( *this, &WorldInfoViewer::update ), 200 );
}


/** Destructor. */
WorldInfoViewer::~WorldInfoViewer()
{
  delete m_field_view;
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
    if (asprintf(&err_str, "Couldn't find widget %s", widget_name) != -1)
    {
      throw std::runtime_error(err_str);
      free(err_str);
    } 
    else 
    { throw std::runtime_error("Getting widget failed"); }
  }

  return widget;
}


/** Update the GUI */
bool
WorldInfoViewer::update()
{
  bool robot_removed = false;

  if ( m_data_container->check_timeout() )
  {
    robot_removed = true;
    list<string> timedout_hosts = m_data_container->get_timedout_hosts();

#ifdef DEBUG_PRINT
    printf( "Removing %zu timed out host.\n", timedout_hosts.size() );
#endif /* DEBUG_PRINT */
    
    // remove timed out hosts
    for ( list<string>::iterator hit = timedout_hosts.begin();
	  hit != timedout_hosts.end();
	  ++hit )
    {
      m_field_view->remove_host( Glib::ustring( *hit ) );

      Gtk::TreeModel::Children children = m_robots_list->children();
      Gtk::TreeModel::iterator cit = children.begin();
      while ( cit != children.end() )
      {
	Gtk::TreeModel::Row row = *cit;
	if ( Glib::ustring( *hit ) == row[ m_robot_record.fqdn ] )
	{ cit = m_robots_list->erase( cit ); }
	else
	{ ++cit; }
      }
    }
  }

  // return if no new data is available
  if ( !m_data_container->new_data_available() )
  {
    if ( robot_removed )
    { m_field_view->queue_draw(); }
    return true;
  }

  list<string> hosts = m_data_container->get_hosts();

  // check that all hosts are in the treeview
  for ( list<string>::iterator hit = hosts.begin();
	hit != hosts.end();
	++hit )
  {
    bool found = false;
    
    Gtk::TreeModel::Children children = m_robots_list->children();
    for ( Gtk::TreeModel::iterator i = children.begin();
	  i != children.end();
	  ++i )
    {
      Gtk::TreeModel::Row row = *i;
      if ( Glib::ustring( *hit ) == row[ m_robot_record.fqdn ] )
      { 
	found = true;
	break;
      }
    }
    
    if ( !found )
    {
      char* fqdn;
      char* hostname;
      char delim ='.';
      Glib::ustring fqdn_str = Glib::ustring( *hit );

      fqdn = strdup( hit->c_str() );
      hostname = strtok( fqdn, &delim );
      int i = atoi( hostname );
      
      Gtk::TreeModel::Row row = *m_robots_list->append();
    
      if ( 0 == i ) /* fqdn is not an IP address */
      { row[ m_robot_record.hostname ] = Glib::ustring( hostname ); }
      else
      { row[ m_robot_record.hostname ] = fqdn_str; }
      row[ m_robot_record.fqdn ]           = fqdn_str;
      row[ m_robot_record.show_pose ]      = m_field_view->toggle_show_pose( fqdn_str );
      row[ m_robot_record.show_ball ]      = m_field_view->toggle_show_ball( fqdn_str );
      row[ m_robot_record.show_opponents ] = m_field_view->toggle_show_opponents( fqdn_str );
      
      free(fqdn);
    }
  }
  
  m_field_view->queue_draw();

  return true;
}

/** Call this method whenever the game state changes. */
void
WorldInfoViewer::gamestate_changed()
{
  char* status_string;
  if ( asprintf( &status_string, 
		 "Team color: %s  Goal color: %s  Mode: %s  Score: %d:%d  Half: %s",
		 m_data_container->get_own_team_color_string().c_str(),
		 m_data_container->get_own_goal_color_string().c_str(),
		 m_data_container->get_game_state_string().c_str(),
		 m_data_container->get_own_score(),
		 m_data_container->get_other_score(),
		 m_data_container->get_half_string().c_str() ) != -1 )
  {
    m_stb_status->remove_message(m_stb_message_id);
    m_stb_message_id = m_stb_status->push( Glib::ustring(status_string) );
    
    free(status_string);
  }
}

void
WorldInfoViewer::on_show_pose_toggled( const Glib::ustring& path )
{
  Gtk::TreeModel::Row row = *m_robots_list->get_iter( path );
  Glib::ustring fqdn = row[ m_robot_record.fqdn ];
  
  row[ m_robot_record.show_pose ] = m_field_view->toggle_show_pose( fqdn );
  
  m_field_view->queue_draw();
}

void
WorldInfoViewer::on_show_ball_toggled( const Glib::ustring& path )
{
  Gtk::TreeModel::Row row = *m_robots_list->get_iter( path );
  Glib::ustring fqdn = row[ m_robot_record.fqdn ];
  
  row[ m_robot_record.show_ball ] = m_field_view->toggle_show_ball( fqdn );
  
  m_field_view->queue_draw();
}

void
WorldInfoViewer::on_show_opponents_toggled( const Glib::ustring& path )
{
  Gtk::TreeModel::Row row = *m_robots_list->get_iter( path );
  Glib::ustring fqdn = row[ m_robot_record.fqdn ];
  
  row[ m_robot_record.show_opponents ] = m_field_view->toggle_show_opponents( fqdn );
  
  m_field_view->queue_draw();
}
