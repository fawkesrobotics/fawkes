
/***************************************************************************
 *  plugin_gui.h - Plugin Tool Gui
 *
 *  Created: Thu Nov 09 20:15:27 2007
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

#ifndef __TOOLS_PLUGIN_PLUGIN_GUI_PLUGIN_GUI_H_
#define __TOOLS_PLUGIN_PLUGIN_GUI_PLUGIN_GUI_H_

#include <gtkmm.h>

namespace fawkes {
  class PluginTreeView;
  class ServiceSelectorCBE;
}

#define GCONF_PREFIX "/apps/fawkes/plugingui"

class PluginGuiGtkWindow : public Gtk::Window
{
 public:
  PluginGuiGtkWindow(BaseObjectType* cobject,
                     const Glib::RefPtr<Gtk::Builder> builder);
  virtual ~PluginGuiGtkWindow();

 private:
  void on_connect();
  void on_disconnect();

 private:
  Gtk::Statusbar             *m_stb_status;
  fawkes::PluginTreeView     *m_trv_plugins;
  fawkes::ServiceSelectorCBE *m_service_selector;
};

#endif /* __TOOLS_PLUGIN_PLUGIN_GUI_PLUGIN_GUI_H_ */
