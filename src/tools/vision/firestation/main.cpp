
/***************************************************************************
 *  main.cpp - Firestation
 *
 *  Created: Wed Oct 10 14:11:05 2007
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

#include "firestation.h"
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
  try
  {
    Gtk::Main kit(argc, argv);
    Glib::RefPtr<Gtk::Builder> builder =
      Gtk::Builder::create_from_file(RESDIR"/guis/firestation/firestation.ui");
    Firestation firestation(builder);
    kit.run(firestation.get_window());
    return 0;
  }
  catch (std::exception const& e)
  {
    std::cerr << "Error: " << e.what() << std::endl;
    return -1;
  }
}
