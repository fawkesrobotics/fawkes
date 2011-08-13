
/***************************************************************************
 *  main.cpp - Fawkes Battery Monitor
 *
 *  Created: Mon Apr 06 16:55:15 2009
 *  Copyright  2009  Daniel Beck
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

#include "battery_monitor.h"
#include <iostream>

using namespace std;

int main( int argc, char** argv )
{
  try
  {
    Gtk::Main kit( argc, argv );
    Glib::RefPtr<Gtk::Builder> builder =
      Gtk::Builder::create_from_file( RESDIR"/guis/battery_monitor/battery_monitor.ui");

    BatteryMonitor battery_monitor(builder);

    kit.run(battery_monitor.get_window());
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
