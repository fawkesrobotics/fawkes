
/***************************************************************************
 *  main.cpp - Firestation
 *
 *  Created: Wed Oct 10 14:11:05 2007
 *  Copyright  2007  Daniel Beck
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <tools/firestation/firestation.h>
#include <libglademm/xml.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
  try
    {
      Gtk::Main kit(argc, argv);
      Glib::RefPtr<Gnome::Glade::Xml> refXml = Gnome::Glade::Xml::create(RESDIR"/glade/firestation/firestation.glade");
      Firestation firestation(refXml);
      kit.run( firestation.get_window() );
      return 0;
    }
  catch (std::exception const& e)
    {
      std::cerr << "Error: " << e.what() << std::endl;
      return -1;
    }
}
