
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

#include <core/exception.h>
#include <tools/plugin_gui/plugin_gui.h>
#ifdef HAVE_GCONFMM
#  include <gconfmm.h>
#endif
#include <libglademm/xml.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
  try
  {
    Gtk::Main kit(argc, argv);
#ifdef HAVE_GCONFMM
    Gnome::Conf::init();
#endif

#ifdef GLIBMM_EXCEPTIONS_ENABLED
    Glib::RefPtr<Gnome::Glade::Xml> refxml = Gnome::Glade::Xml::create(RESDIR"/glade/plugin_tool/plugin_tool.glade");
#else
    std::auto_ptr<Gnome::Glade::XmlError> error;
    Glib::RefPtr<Gnome::Glade::Xml> refxml = Gnome::Glade::Xml::create(RESDIR"/glade/plugin_tool/plugin_tool.glade", "", "", error);
    if (error.get()) {
      throw fawkes::Exception("Failed to load Glade file: %s", error->what().c_str());
    }
#endif

    PluginGuiGtkWindow *window = NULL;
    refxml->get_widget_derived("wndMain", window);

    kit.run( *window );

    delete window;
  }
  catch (std::exception const& e)
  {
    std::cerr << "Error: " << e.what() << std::endl;
  }

  return 0;
}
