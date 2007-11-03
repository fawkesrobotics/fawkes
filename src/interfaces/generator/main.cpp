 
/***************************************************************************
 *  main.cpp - Interface generator main app
 *
 *  Generated: Tue Oct 10 17:42:05 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <interfaces/generator/generator.h>
#include <interfaces/generator/parser.h>
#include <interfaces/generator/exceptions.h>

#include <utils/system/argparser.h>
#include <utils/system/file.h>

#include <iostream>
#include <vector>
#include <string>

using namespace std;

int
main(int argc, char **argv)
{
  ArgumentParser *argp = new ArgumentParser(argc, argv, "d:");

  vector<char *> & items = argp->getItems();
  if ( items.size() == 0 ) {
    cout << "Usage: " << argv[0] << " [-d dir] config.xml [config2.xml...]" << endl << endl;
  } else {
    string dir = INTERFACEDIR;
    if ( argp->hasArgument("d") ) {
      dir = argp->getArgument("d");
    }

    for ( vector<char *>::iterator i = items.begin(); i != items.end(); ++i) {
      string s = *i;
      string prefix;
      size_t pos;

      if ( ( pos = s.find_last_of (".")) != string::npos ) {
	prefix = s.substr(0, pos);
      } else {
	prefix = s;
      }
      s = prefix;
      if ( ( pos = s.find_last_of ("/")) != string::npos ) {
	prefix = s.substr(pos + 1);
      } else {
	prefix = s;
      }

      if ( ! File::exists( *i ) ) {
	cout << "File " << *i << " does not exist" << endl;
	continue;
      } else if (! File::is_regular( *i ) ) {
	cout << *i << " is not a regular file" << endl;
	continue;
      }

      try {
	InterfaceParser    *iparse = new InterfaceParser(*i);
	iparse->parse();
	// iparse->print();
	
	InterfaceGenerator *igen = new InterfaceGenerator(dir,
							  iparse->getInterfaceName(),
							  prefix,
							  iparse->getInterfaceAuthor(),
							  iparse->getInterfaceYear(),
							  iparse->getInterfaceCreationDate(),
							  iparse->getDataComment()
							  );
	igen->setConstants( iparse->getConstants() );
	igen->setEnumConstants( iparse->getEnumConstants() );
	igen->setDataFields( iparse->getDataFields() );
	igen->setMessages( iparse->getMessages() );

	igen->generate();
	delete igen;
	delete iparse;
      } catch (Exception &e) {
	cout << "Generating the interface failed." << endl;
	e.print_trace();
      }
    }
  }

  delete argp;
}
