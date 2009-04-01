
/***************************************************************************
 *  main.cpp - YUV viewer gui
 *
 *  Created:  Sat Mar 22 16:34:02 2009
 *  Copyright 2009 Christof Rath <c.rath@student.tugraz.at>
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

#include "yuv_viewer_gui.h"

#include <core/exception.h>

#include <libglademm/xml.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
  Glib::thread_init();

  try
  {
    Gtk::Main kit(argc, argv);

#ifdef GLIBMM_EXCEPTIONS_ENABLED
    Glib::RefPtr<Gnome::Glade::Xml> refxml = Gnome::Glade::Xml::create(RESDIR"/glade/yuv_viewer/yuv_viewer.glade");
#else
    std::auto_ptr<Gnome::Glade::XmlError> error;
    Glib::RefPtr<Gnome::Glade::Xml> refxml = Gnome::Glade::Xml::create(RESDIR"/glade/yuv_viewer/yuv_viewer.glade", "", "", error);
    if (error.get()) {
      throw fawkes::Exception("Failed to load Glade file: %s", error->what().c_str());
    }
#endif

    YuvViewerGtkWindow *window = NULL;
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
