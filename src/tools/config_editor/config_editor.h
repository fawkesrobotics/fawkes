
/***************************************************************************
 *  config_editor.h - Fawkes Config Editor
 *
 *  Created: Tue Sep 23 13:17:41 2008
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

#ifndef __TOOLS_CONFIG_EDITOR_CONFIG_EDITOR_H_
#define __TOOLS_CONFIG_EDITOR_CONFIG_EDITOR_H_

#include <gtkmm.h>
#include <libglademm/xml.h>

class ConfigTreeView;

namespace fawkes{
  class ServiceSelectorCBE;
  class FawkesNetworkClient;
}

class FawkesConfigEditor 
: public Gtk::Window
{
 public:
  FawkesConfigEditor( Glib::RefPtr<Gnome::Glade::Xml> ref_xml );
  ~FawkesConfigEditor();

  Gtk::Window& get_window() const;

 private:
  void on_btn_exit_clicked();
  void on_connected();
  void on_disconnected();

  fawkes::ServiceSelectorCBE* m_service_selector;
  fawkes::FawkesNetworkClient* m_network_client;

  Gtk::Window* m_wnd_main;
  Gtk::Button* m_btn_exit;
  ConfigTreeView* m_trv_config;
};

#endif /* __TOOLS_CONFIG_EDITOR_CONFIG_EDITOR_H_ */
