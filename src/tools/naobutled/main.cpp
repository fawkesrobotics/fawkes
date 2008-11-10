
/***************************************************************************
 *  main.cpp - Nao ButLed main
 *
 *  Created: Thu Oct 30 21:11:39 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#include "naobutled.h"

/** This is the main program of the Nao GUI.
 */
int
main(int argc, char **argv) {
  Gtk::Main gtk_main(argc, argv);

  Glib::RefPtr<Gnome::Glade::Xml> refxml;
  refxml = Gnome::Glade::Xml::create(RESDIR"/glade/nao/butled.glade");

  NaoButLedGtkWindow *window = NULL;
  refxml->get_widget_derived("wnd_butled", window);

  Gtk::Main::run(*window);

  delete window;

  return 0;
}
