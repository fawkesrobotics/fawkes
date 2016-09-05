
/***************************************************************************
 *  main.cpp - Plugin Tool Gui
 *
 *  Created: Thu Nov 09 20:13:45 2007
 *  Copyright  2007  Daniel Beck
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

#include <core/exception.h>
#include <tools/plugin_gui/plugin_gui.h>
#ifdef HAVE_GCONFMM
#  include <gconfmm.h>
#endif
#include <iostream>

#if GTK_VERSION_GE(3,0)
#  define UI_FILE RESDIR"/guis/plugin_tool/plugin_gui.ui"
#else
#  define UI_FILE RESDIR"/guis/plugin_tool/plugin_gui_gtk2.ui"
#endif


using namespace std;

int main(int argc, char** argv)
{
  try
  {
    Gtk::Main kit(argc, argv);
#ifdef HAVE_GCONFMM
    Gnome::Conf::init();
#endif

    Glib::RefPtr<Gtk::Builder> builder;
    try {
      builder =
        Gtk::Builder::create_from_file(UI_FILE);
    } catch (Gtk::BuilderError &e) {
      printf("Failed to create GUI: %s\n", e.what().c_str());
    }

    PluginGuiGtkWindow *window = NULL;
    builder->get_widget_derived("wndMain", window);

    kit.run( *window );

    delete window;
  }
  catch (std::exception const& e)
  {
    std::cerr << "Error: " << e.what() << std::endl;
  }

  return 0;
}
