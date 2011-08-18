
/***************************************************************************
 *  main.cpp - Fawkes Config Editor
 *
 *  Created: Tue Sep 23 13:26:18 2008
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

#include <core/exception.h>
#include <tools/config_editor/config_editor.h>
#include <iostream>

#if GTK_VERSION_GE(3,0)
#  define UI_FILE RESDIR"/guis/config_editor/config_editor.ui"
#else
#  define UI_FILE RESDIR"/guis/config_editor/config_editor_gtk2.ui"
#endif

using namespace std;

int main(int argc, char** argv)
{
  std::locale::global( std::locale( "" ) ); 

  try
  {
    Gtk::Main kit(argc, argv);
#ifdef GLIBMM_EXCEPTIONS_ENABLED
    Glib::RefPtr<Gtk::Builder> builder =
      Gtk::Builder::create_from_file(UI_FILE);
#else
    std::auto_ptr<Gtk::BuilderError> error;
    Glib::RefPtr<Gtk::Builder> builder =
      Gtk::Builder::create_from_file(UI_FILE, error);
    if (error.get()) {
      printf("Failed to load UI file: %s", error->what().c_str());
      exit(-1);
    }
#endif

    FawkesConfigEditor fce(builder);

    kit.run(fce.get_window());
  }
  catch (Gtk::BuilderError &e)
  {
    printf("Failed to instantiate window: %s\n", e.what().c_str());
  }
  catch (const std::exception& e)
  {
    cerr << "Error: " << e.what() << endl;
  }

  return 0;
}
