
/***************************************************************************
 *  plugin_gui.cpp -  Plugin Tool Gui
 *
 *  Created: Thu Nov 09 20:16:23 2007
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

#include <tools/plugin_gui/plugin_gui.h>
#include <tools/plugin_gui/plugin_tree_view.h>
#include <gui_utils/service_selector_cbe.h>
#include <gui_utils/utils.h>

using namespace fawkes;

/** @class PluginGui tools/plugin_gui/plugin_gui.h
 * Graphical plugin management tool.
 * 
 * @author Daniel Beck
 */

/** Constructor.
 * @param ref_xml RefPtr to the Glade xml file
 */
PluginGui::PluginGui(Glib::RefPtr<Gnome::Glade::Xml> ref_xml)
{
  m_wnd_main    = dynamic_cast<Gtk::Window*>( get_widget(ref_xml, "wndMain") );
  m_stb_status  = dynamic_cast<Gtk::Statusbar*>( get_widget(ref_xml, "stbStatus") );
  
  ref_xml->get_widget_derived("trvPlugins", m_trv_plugins);

  m_service_selector = new ServiceSelectorCBE(ref_xml, "cbeHosts", "btnConnect");
  m_service_selector->signal_connected().connect( sigc::mem_fun(*this, &PluginGui::on_connection_established) );

  m_stb_status->push("Started");
}

/** Destructor. */
PluginGui::~PluginGui()
{
  m_stb_status->push("Exiting");
}

/** Return main window of the application.
 * @return reference to the main window
 */
Gtk::Window&
PluginGui::get_window() const
{
  return *m_wnd_main;
}

void
PluginGui::on_connection_established()
{
  m_trv_plugins->set_network_client( m_service_selector->get_network_client() );
}
