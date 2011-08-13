
/***************************************************************************
 *  plugin_gui.cpp -  Plugin Tool Gui
 *
 *  Created: Thu Nov 09 20:16:23 2007
 *  Copyright  2007       Daniel Beck
 *             2008-2009  Tim Niemueller [www.niemueller.de]
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

#include <string>

using namespace fawkes;

/** @class PluginGuiGtkWindow "plugin_gui.h"
 * Graphical plugin management tool.
 *
 * @author Daniel Beck
 * @author Tim Niemueller
 */

/** Constructor.
 * @param cobject C base object
 * @param builder Gtk Builder
 */
PluginGuiGtkWindow::PluginGuiGtkWindow(BaseObjectType* cobject,
				       const Glib::RefPtr<Gtk::Builder> builder)
  : Gtk::Window(cobject)
{
  builder->get_widget("stbStatus", m_stb_status);
  builder->get_widget_derived("trvPlugins", m_trv_plugins);

#ifdef HAVE_GCONFMM
  m_trv_plugins->set_gconf_prefix(GCONF_PREFIX);
#endif

  m_service_selector = new ServiceSelectorCBE(builder, "cbeHosts", "btnConnect", "wndMain");
  m_trv_plugins->set_network_client( m_service_selector->get_network_client() );

  m_service_selector->signal_connected().connect(sigc::mem_fun(*this, &PluginGuiGtkWindow::on_connect));
  m_service_selector->signal_disconnected().connect(sigc::mem_fun(*this, &PluginGuiGtkWindow::on_disconnect));

  m_stb_status->push("Started");
}

/** Destructor. */
PluginGuiGtkWindow::~PluginGuiGtkWindow()
{
  m_stb_status->push("Exiting");
}

/** Connected handler. */
void
PluginGuiGtkWindow::on_connect()
{
  this->set_title(std::string("Fawkes Plugin Tool @ ") + m_service_selector->get_name());
}

/** Disconnected handler. */
void
PluginGuiGtkWindow::on_disconnect()
{
  this->set_title("Fawkes Plugin Tool");
}
