
/***************************************************************************
 *  config_editor.cpp - Fawkes Config Editor
 *
 *  Created: Tue Sep 23 13:21:49 2008
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

#include "config_editor.h"
#include "config_tree_view.h"
#include "retriever_config_plugin.h"
#include "naostiffness_config_plugin.h"

#include <gui_utils/service_selector_cbe.h>
#include <netcomm/fawkes/client.h>

#include <cstdlib>
#include <cstring>
#include <iostream>

using namespace std;
using namespace fawkes;

/** @class FawkesConfigEditor "config_editor.h"
 * Graphical configuration editor.
 *
 * @author Daniel Beck
 */

/** Constructor.
 * @param builder Gtk builder
 */
FawkesConfigEditor::FawkesConfigEditor(Glib::RefPtr<Gtk::Builder> builder)
{
  builder->get_widget("wndMain", m_wnd_main);
  builder->get_widget("btnExit", m_btn_exit);

  m_trv_config = NULL;
  builder->get_widget_derived("trvConfig", m_trv_config);
  m_trv_config->register_plugin( new RetrieverConfigPlugin( RESDIR"/guis/config_editor/retriever_config_plugin.ui" ) );
  m_trv_config->register_plugin(new NaoStiffnessConfigPlugin(RESDIR"/guis/config_editor/naostiffness_config_plugin.ui"));

  m_btn_exit->signal_clicked().connect( sigc::mem_fun( *this, &FawkesConfigEditor::on_btn_exit_clicked) );

  m_service_selector = new ServiceSelectorCBE(builder, "cbeHosts", "btnConnect");
  m_service_selector->signal_connected().connect(sigc::mem_fun(*this, &FawkesConfigEditor::on_connected));
  m_service_selector->signal_disconnected().connect(sigc::mem_fun( *this, &FawkesConfigEditor::on_disconnected));
}

/** Destructor. */
FawkesConfigEditor::~FawkesConfigEditor()
{
  delete m_service_selector;
}

/** Obtain a reference to the main window of the application.
 * @return reference to the main window
 */
Gtk::Window&
FawkesConfigEditor::get_window() const
{
  return *m_wnd_main;
}

void
FawkesConfigEditor::on_btn_exit_clicked()
{
  m_wnd_main->hide();
}

void
FawkesConfigEditor::on_connected()
{
  m_network_client = m_service_selector->get_network_client();
  m_trv_config->set_network_client( m_network_client );
  m_wnd_main->set_title("Fawkes Config Editor @ " + m_service_selector->get_name());
}

void
FawkesConfigEditor::on_disconnected()
{
  m_trv_config->set_network_client( NULL );
  m_wnd_main->set_title("Fawkes Config Editor");
}
