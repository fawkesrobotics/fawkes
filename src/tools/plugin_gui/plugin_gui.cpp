
/***************************************************************************
 *  plugin_gui.cpp -  Plugin Tool Gui
 *
 *  Created: Thu Nov 09 20:16:23 2007
 *  Copyright  2007       Daniel Beck
 *             2008-2009  Tim Niemueller [www.niemueller.de]
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

#include "plugin_gui.h"
#include <gui_utils/plugin_tree_view.h>
#include <gui_utils/service_selector_cbe.h>

using namespace fawkes;

/** @class PluginGuiGtkWindow "plugin_gui.h"
 * Graphical plugin management tool.
 *
 * @author Daniel Beck
 * @author Tim Niemueller
 */

/** Constructor.
 * @param cobject C base object
 * @param ref_xml Glade XML
 */
PluginGuiGtkWindow::PluginGuiGtkWindow(BaseObjectType* cobject,
				       const Glib::RefPtr<Gnome::Glade::Xml> ref_xml)
  : Gtk::Window(cobject)
{
  ref_xml->get_widget("stbStatus", m_stb_status);
  ref_xml->get_widget_derived("trvPlugins", m_trv_plugins);

  m_trv_plugins->set_gconf_prefix(GCONF_PREFIX);

  m_service_selector = new ServiceSelectorCBE(ref_xml, "cbeHosts", "btnConnect", "wndMain");
  m_trv_plugins->set_network_client( m_service_selector->get_network_client() );

  m_stb_status->push("Started");
}

/** Destructor. */
PluginGuiGtkWindow::~PluginGuiGtkWindow()
{
  m_stb_status->push("Exiting");
}
