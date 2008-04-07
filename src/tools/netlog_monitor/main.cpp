
/***************************************************************************
 *  main.cpp - Network Log Monitor
 *
 *  Created: Sun Dec 09 20:13:01 2007
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

#include <tools/netlog_monitor/netlog_monitor.h>
#include <tools/netlog_monitor/backend_thread.h>
#include <libglademm/xml.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
  NetLogMonitorBackendThread* backend = 0;

  Thread::init_main();

  try
    {
      Gtk::Main kit(argc, argv);
      Glib::RefPtr<Gnome::Glade::Xml> ref_xml = Gnome::Glade::Xml::create(RESDIR"/glade/netlog_monitor/netlog_monitor.glade");
      NetLogMonitor netlog(ref_xml);
      backend = new NetLogMonitorBackendThread(&netlog);
      netlog.register_backend(backend);
      kit.run( netlog.get_window() );
    }
  catch (std::exception const& e)
    {
      std::cerr << "Error: " << e.what() << std::endl;
    }

  Thread::destroy_main();

  return 0;
}
