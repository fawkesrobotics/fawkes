
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
#include <tools/worldinfo_viewer/data_container.h>
#include <vector>
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

  m_data_container = data_container;
}

/** Destructor. */
WorldInfoViewer::~WorldInfoViewer()
{
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

  vector<string>::iterator hit;
  vector<string> hosts = m_data_container->get_hosts();

  HomVector pos;
  for (hit = hosts.begin(); hit != hosts.end(); ++hit)
    {
      pos = m_data_container->get_robot_pos( hit->c_str() );
      m_field_view->set_robot_pos( hit->c_str(), pos.x(), pos.y(), 0.0 /* TODO */);
      //m_field_view->set_ball_pos();
    }
  
  m_field_view->queue_draw();
}
