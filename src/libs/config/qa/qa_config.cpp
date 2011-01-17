
/***************************************************************************
 *  qa_config.h - QA for configuration storage
 *
 *  Generated: Mon Dec 18 19:09:18 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

/// @cond QA

#include <config/sqlite.h>

#include <iostream>
#include <cstdio>

using namespace std;
using namespace fawkes;

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
    e.print_trace();
  }

  try {
    float of = 5.234;
    cout << "[FLOAT] set f=" << of << "..." << flush;
    config->set_float("/testing/float", of);
    cout << "done" << endl;
    cout << "[FLOAT] get..." << flush;
    float f = config->get_float("/testing/float");
    printf("done, f=%f\n", f);
  } catch (ConfigurationException &e) {
    cout << "failed" << endl;
    e.print_trace();
  }

  try {
    float of = 5.234;
    cout << "[DEFAULT FLOAT] set f=" << of << "..." << flush;
    config->set_default_float("/testing/default_float", of);
    cout << "done" << endl;
    cout << "[DEFAULT_FLOAT] get..." << flush;
    float f = config->get_float("/testing/default_float");
    if ( ! config->is_default("/testing/default_float") ) {
      throw ConfigurationException("/testing/default_float is not in default config");
    }
    printf("done, f=%f\n", f);
  } catch (ConfigurationException &e) {
    cout << "failed" << endl;
    e.print_trace();
  }

  try {
    unsigned int ou = 6;
    cout << "[UINT] set u=" << ou << "..." << flush;
    config->set_uint("/testing/uint", ou);
    cout << "done" << endl;
    cout << "[UINT] get..." << flush;
    unsigned int u = config->get_uint("/testing/uint");
    printf("done, u=%u\n", u);
  } catch (ConfigurationException &e) {
    cout << "failed" << endl;
    e.print_trace();
  }

  try {
    int oi = -7;
    cout << "[INT] set i=" << oi << "..." << flush;
    config->set_int("/testing/int", oi);
    cout << "done" << endl;
    cout << "[INT] get..." << flush;
    int i = config->get_int("/testing/int");
    printf("done, i=%i\n", i);
  } catch (ConfigurationException &e) {
    cout << "failed" << endl;
    e.print_trace();
  }

  try {
    bool ob = true;
    cout << "[BOOL] set b=" << ob << "..." << flush;
    config->set_bool("/testing/bool", ob);
    cout << "done" << endl;
    cout << "[BOOL] get..." << flush;
    bool b = config->get_bool("/testing/bool");
    printf("done, b=%s\n", (b ? "true" : "false"));
  } catch (ConfigurationException &e) {
    cout << "failed" << endl;
    e.print_trace();
  }

  try {
    string os = "This ain't no paradoxon";
    cout << "[STRING] set s='" << os << "'..." << flush;
    config->set_string("/testing/string", os);
    cout << "done" << endl;
    cout << "[STRING] get..." << flush;
    string s = config->get_string("/testing/string");
    printf("done, s='%s'\n", s.c_str());
  } catch (ConfigurationException &e) {
    cout << "failed" << endl;
    e.print_trace();
  }

  try {
    cout << "[EXIST] Checking if test string exists..." << flush;
    if ( config->exists("/testing/string") ) {
      cout << "success";
    } else {
      cout << "failed";
    }
    cout << endl;
  } catch (ConfigurationException &e) {
    cout << "failed" << endl;
    e.print_trace();
  }

  try {
    string os = "This ain't no paradoxon";
    cout << "[LONGSTRING] set s='" << os << "'..." << flush;
    config->set_string("/testing/veryveryveryverylongstring", os);
    cout << "done" << endl;
    cout << "[LONGSTRING] get..." << flush;
    string s = config->get_string("/testing/veryveryveryverylongstring");
    printf("done, s='%s'\n", s.c_str());
  } catch (ConfigurationException &e) {
    cout << "failed" << endl;
    e.print_trace();
  }

  Configuration::ValueIterator *i = config->iterator();
  while (i->next()) {
    if ( i->is_float() ) {
      printf("FLOAT: %s = %f (default: %i)\n", i->path(), i->get_float(), i->is_default());
    }
  }

  SQLiteConfiguration *config2 = new SQLiteConfiguration(CONFDIR);

  try {
    cout << "Loading configuration for 2nd db..." << flush;
    config2->load("qa2.db", "qa2_defaults.db");
    cout << "done" << endl;
  } catch (CouldNotOpenConfigException &e) {
    cout << "failed" << endl;
    e.print_trace();
  }

  try {
    cout << "Copying configuration..." << flush;
    config2->copy(config);
    cout << "done" << endl;
  } catch (ConfigurationException &e) {
    cout << "failed" << endl;
    e.print_trace();
  }

  delete config2;
  delete config;

  return 0;
}



/// @endcond
