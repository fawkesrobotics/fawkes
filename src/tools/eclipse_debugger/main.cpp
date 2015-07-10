
/***************************************************************************
 *  main.cpp - Eclipse Debugger Tool main
 *
 *  Created: Mon Feb 25 14:22:00 2013
 *  Copyright  2013  Gesche Gierse
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

#include "eclipse_debugger.h"

#include <glibmm/exception.h>
#ifdef HAVE_GCONFMM
#include <gui_utils/service_chooser_dialog.h>
#endif


#  define UI_FILE RESDIR"/eclipsedebuggergui/eclipsedebuggergui.ui"


/** This is the main program of the Eclipse debugger starter.
 */
int
main(int argc, char **argv) {
  Gtk::Main gtk_main(argc, argv);
#ifdef HAVE_GCONFMM
  Gnome::Conf::init();
#endif

  try {
    Glib::RefPtr<Gtk::Builder> builder =
      Gtk::Builder::create_from_file(UI_FILE);

    EclipseDebugger *window = NULL;
    builder->get_widget_derived("wnd_eclipsedebuggergui", window);

    Gtk::Main::run(*window);

    delete window;
  } catch (Gtk::BuilderError &e) {
    printf("Failed to instantiate window: %s\n", e.what().c_str());
    return 1;
  } catch (Glib::Exception &e) {
    printf("%s\n", e.what().c_str());
    return 1;
  }

  return 0;
}
