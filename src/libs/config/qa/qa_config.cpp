
/***************************************************************************
 *  qa_config.h - QA for configuration storage
 *
 *  Generated: Mon Dec 18 19:09:18 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

/// @cond QA

#include <config/sqlite.h>

#include <iostream>

using namespace std;

int
main(int argc, char **argv)
{
  SQLiteConfiguration *config = new SQLiteConfiguration(CONFDIR);

  try {
    cout << "Loading configuration..." << flush;
    config->load("qa.db", "qa_defaults.db");
    cout << "done" << endl;
  } catch (CouldNotOpenConfigException &e) {
    cout << "failed" << endl;
    e.printTrace();
  }

  try {
    float of = 5.234;
    cout << "[FLOAT] set f=" << of << "..." << flush;
    config->set_float("configqa", "/testing/float", of);
    cout << "done" << endl;
    cout << "[FLOAT] get..." << flush;
    float f = config->get_float("configqa", "/testing/float");
    printf("done, f=%f\n", f);
  } catch (ConfigurationException &e) {
    cout << "failed" << endl;
    e.printTrace();
  }

  try {
    unsigned int ou = 6;
    cout << "[UINT] set u=" << ou << "..." << flush;
    config->set_uint("configqa", "/testing/uint", ou);
    cout << "done" << endl;
    cout << "[UINT] get..." << flush;
    unsigned int u = config->get_uint("configqa", "/testing/uint");
    printf("done, u=%u\n", u);
  } catch (ConfigurationException &e) {
    cout << "failed" << endl;
    e.printTrace();
  }

  try {
    int oi = -7;
    cout << "[INT] set i=" << oi << "..." << flush;
    config->set_int("configqa", "/testing/int", oi);
    cout << "done" << endl;
    cout << "[INT] get..." << flush;
    int i = config->get_int("configqa", "/testing/int");
    printf("done, i=%i\n", i);
  } catch (ConfigurationException &e) {
    cout << "failed" << endl;
    e.printTrace();
  }

  try {
    bool ob = true;
    cout << "[BOOL] set b=" << ob << "..." << flush;
    config->set_bool("configqa", "/testing/bool", ob);
    cout << "done" << endl;
    cout << "[BOOL] get..." << flush;
    bool b = config->get_bool("configqa", "/testing/bool");
    printf("done, b=%s\n", (b ? "true" : "false"));
  } catch (ConfigurationException &e) {
    cout << "failed" << endl;
    e.printTrace();
  }

  try {
    string os = "This ain't no paradoxon";
    cout << "[STRING] set s='" << os << "'..." << flush;
    config->set_string("configqa", "/testing/string", os);
    cout << "done" << endl;
    cout << "[STRING] get..." << flush;
    string s = config->get_string("configqa", "/testing/string");
    printf("done, s='%s'\n", s.c_str());
  } catch (ConfigurationException &e) {
    cout << "failed" << endl;
    e.printTrace();
  }

  try {
    cout << "[EXIST] Checking if test string exists..." << flush;
    if ( config->exists("configqa", "/testing/string") ) {
      cout << "success";
    } else {
      cout << "failed";
    }
    cout << endl;
  } catch (ConfigurationException &e) {
    cout << "failed" << endl;
    e.printTrace();
  }

  delete config;

  return 0;
}



/// @endcond
