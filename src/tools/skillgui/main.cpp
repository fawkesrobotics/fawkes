
/***************************************************************************
 *  main.cpp - Skill GUI main
 *
 *  Created: Mon Nov 30 13:33:13 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#include "skillgui.h"

#if GTK_VERSION_GE(3,0)
#  define UI_FILE RESDIR"/guis/skillgui/skillgui.ui"
#else
#  if GTKMM_VERSION_GE(2,20)
#    define UI_FILE RESDIR"/guis/skillgui/skillgui_gtk2.ui"
#  else
#    define UI_FILE RESDIR"/guis/skillgui/skillgui_gtk2_nospinner.ui"
#  endif
#endif

/** This is the main program of the Skill GUI.
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

    SkillGuiGtkWindow *window = NULL;
    builder->get_widget_derived("wnd_skillgui", window);

    Gtk::Main::run(*window);

    delete window;
  } catch (Gtk::BuilderError &e) {
    printf("Failed to instantiate window: %s\n", e.what().c_str());
  }

  return 0;
}
