 
/***************************************************************************
 *  main.cpp - Interface generator main app
 *
 *  Generated: Tue Oct 10 17:42:05 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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
using namespace fawkes;

int
main(int argc, char **argv)
{
  int rv = 0;
  ArgumentParser *argp = new ArgumentParser(argc, argv, "hd:v");

  const vector<const char *> & items = argp->items();
  if ( items.size() == 0 || argp->has_arg("h") ) {
  cout << "Fawkes Interface generator - Usage Instructions" << endl
       << "===============================================================================" << endl
       << "Usage: " << argv[0] << " [-h] [-d dir] [-v] config.xml [config2.xml...]" << endl
       << "where [options] is one or more of:" << endl
       << " -h        These help instructions" << endl
       << " -d dir    Directory where to write generated files" << endl
       << " -v        Verbose console output." << endl
       << endl;
  } else {
    string dir = ".";
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
	if ( argp->has_arg("v") ) {
	  iparse->print();
	}
	
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
								   iparse->getPseudoMaps(),
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
									 iparse->getPseudoMaps(),
									 iparse->getMessages()
									 );

	cppigen->generate();
	toluaigen->generate();

	delete cppigen;
	delete toluaigen;

	delete iparse;
	delete idigest;
      } catch (Exception &e) {
        cout << "Generating the interface failed: " << e.what_no_backtrace() << endl;
        rv = -1;
      }
    }
  }

  delete argp;

  return rv;
}
