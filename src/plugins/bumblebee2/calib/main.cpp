
/***************************************************************************
 *  main.cpp - BB2 calibration GUI main
 *
 *  Created: Thu Jul 18 20:35:12 2013
 *  Copyright  2008-2013  Tim Niemueller [www.niemueller.de]
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

#include "bb2calib.h"

#include <core/threading/thread.h>

/** This is the main program of the Nao GUI. */
int
main(int argc, char **argv) {
  fawkes::Thread::init_main();

  std::locale::global( std::locale( "" ) );
  Gtk::Main gtk_main(argc, argv);


  try {
    Glib::RefPtr<Gtk::Builder> builder;
    builder =
      Gtk::Builder::create_from_file(RESDIR"/guis/bb2calib/bb2calib.ui");
    
    Bumblebee2CalibGtkWindow *window = NULL;
    builder->get_widget_derived("window", window);
    
    Gtk::Main::run(*window);

    delete window;
  } catch (Gtk::BuilderError &e) {
    printf("Failed to instantiate window: %s\n", e.what().c_str());
  }

  fawkes::Thread::destroy_main();

  return 0;
}
