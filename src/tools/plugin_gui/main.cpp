
/***************************************************************************
 *  main.cpp - Plugin Tool Gui
 *
 *  Created: Thu Nov 09 20:13:45 2007
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
#include <tools/plugin_gui/backend_thread.h>
#include <libglademm/xml.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
  PluginGuiBackendThread* backend = 0;

  Thread::init_main();

  try
    {
      Gtk::Main kit(argc, argv);
      Glib::RefPtr<Gnome::Glade::Xml> refXml = Gnome::Glade::Xml::create(RESDIR"/glade/plugin_tool/plugin_tool.glade");
      PluginGui pt_gui(refXml);
      backend = new PluginGuiBackendThread(&pt_gui);
      pt_gui.register_backend(backend);
      kit.run( pt_gui.get_window() );
    }
  catch (std::exception const& e)
    {
      std::cerr << "Error: " << e.what() << std::endl;
    }

  Thread::destroy_main();

  return 0;
}
