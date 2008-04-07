 
/***************************************************************************
 *  main.cpp - Interface generator main app
 *
 *  Generated: Tue Oct 10 17:42:05 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#include <interfaces/generator/cpp_generator.h>
#include <interfaces/generator/tolua_generator.h>
#include <interfaces/generator/parser.h>
#include <interfaces/generator/digest.h>
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

  const vector<const char *> & items = argp->items();
  if ( items.size() == 0 ) {
    cout << "Usage: " << argv[0] << " [-d dir] config.xml [config2.xml...]" << endl << endl;
  } else {
    string dir = INTERFACEDIR;
    if ( argp->has_arg("d") ) {
      dir = argp->arg("d");
    }

    for ( vector<const char *>::const_iterator i = items.begin(); i != items.end(); ++i) {
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
	
	InterfaceDigest    *idigest = new InterfaceDigest(*i);

	CppInterfaceGenerator *cppigen = new CppInterfaceGenerator(dir,
								   iparse->getInterfaceName(),
								   prefix,
								   iparse->getInterfaceAuthor(),
								   iparse->getInterfaceYear(),
								   iparse->getInterfaceCreationDate(),
								   iparse->getDataComment(),
								   idigest->get_hash(),
								   idigest->get_hash_size(),
								   iparse->getConstants(),
								   iparse->getEnumConstants(),
								   iparse->getDataFields(),
								   iparse->getMessages()
								   );

	ToLuaInterfaceGenerator *toluaigen = new ToLuaInterfaceGenerator(dir,
									 iparse->getInterfaceName(),
									 prefix,
									 iparse->getInterfaceAuthor(),
									 iparse->getInterfaceYear(),
									 iparse->getInterfaceCreationDate(),
									 iparse->getDataComment(),
									 idigest->get_hash(),
									 idigest->get_hash_size(),
									 iparse->getConstants(),
									 iparse->getEnumConstants(),
									 iparse->getDataFields(),
									 iparse->getMessages()
									 );

	cppigen->generate();
	toluaigen->generate();

	delete cppigen;
	delete toluaigen;

	delete iparse;
	delete idigest;
      } catch (Exception &e) {
	cout << "Generating the interface failed." << endl;
	e.print_trace();
      }
    }
  }

  delete argp;
}
