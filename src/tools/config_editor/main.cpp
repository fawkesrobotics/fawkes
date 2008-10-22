
/***************************************************************************
 *  main.cpp - Fawkes Config Editor
 *
 *  Created: Tue Sep 23 13:26:18 2008
 *  Copyright  2008  Daniel Beck
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

#include <core/exception.h>
#include <tools/config_editor/config_editor.h>
#include <libglademm/xml.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
  try
    {
      Gtk::Main kit(argc, argv);
#ifdef GLIBMM_EXCEPTIONS_ENABLED
      Glib::RefPtr<Gnome::Glade::Xml> ref_xml = Gnome::Glade::Xml::create(RESDIR"/glade/config_editor/config_editor.glade");
#else
      std::auto_ptr<Gnome::Glade::XmlError> error;
      Glib::RefPtr<Gnome::Glade::Xml> ref_xml = Gnome::Glade::Xml::create(RESDIR"/glade/config_editor/config_editor.glade", "", "", error);
      if (error.get()) {
        throw fawkes::Exception("Failed to load Glade file: %s", error->what().c_str());
      }
#endif

      FawkesConfigEditor fce(ref_xml);

      kit.run( fce.get_window() );
    }
  catch (const std::exception& e)
    {
      cerr << "Error: " << e.what() << endl;
    }
  
  return 0;
}
