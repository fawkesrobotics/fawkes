/***************************************************************************
 *  clips_test.h - Unit Tests with CLIPS
 *
 *  Created: Tue 19 Sep 2017 13:40:37 CEST 13:40
 *  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include <gtest/gtest.h>
#include <clipsmm.h>

#include <string>
#include <vector>

#ifndef TESTDIR
#define TESTDIR "."
#endif

/** Base class for unit testing with CLIPS.
 * To define your own test setup, derive from this class.
 * @see SimpleCLIPSTest
 */
class CLIPSTest : public ::testing::Test
{
  protected:
    /** The default CLIPS environment used to run tests. */
    CLIPS::Environment env;
    /** Load the vector of CLIPS files into the environment.
     *  @param files A vector of paths relative to the current directory. */
    virtual void LoadCLIPSFiles(std::vector<std::string> files) {
      for (auto & file : files) {
	      std::cout << "[          ] loading file " << file << std::endl;
        const std::string path = std::string(TESTDIR) + "/" + file;
        env.evaluate("(load* " + path + ")");
      }
    }
    /** Check if a non-ordered fact exists.
     *  This expects the same arguments as CLIPS' any-factp.
     *  @param fact_set_template The fact set template of the query,
     *  e.g., "((?a action ?p parameters))".
     *  @param query The constraints that must be satisfied by the fact set,
     *  e.g., "(eq ?a:param-values ?p:values)".
     *  @return true iff the fact exists.
     */
    bool has_fact(const std::string &fact_set_template,
        const std::string &query = "TRUE") {
      const std::string & clips_query =
        "(any-factp " + fact_set_template + " " + query + ")";
      return env.evaluate(clips_query)[0].as_string() == "TRUE";
    }
    /** Check if an ordered fact exists.
     *  @param fact_name The name of the fact, e.g., "foo".
     *  @param slot_values A vector of slot values, e.g., '{ "bar" }'.
     *  @return true iff the specified ordered fact exists.
     */
    bool has_ordered_fact(const std::string &fact_name,
        const std::vector<CLIPS::Value> slot_values = {}) {
      const std::string fact_set_template = "((?f " + fact_name + "))";
      std::string query = "(eq ?f:implied (create$";
      for (CLIPS::Value slot_val : slot_values) {
        switch (slot_val.type()) {
          case CLIPS::TYPE_FLOAT:
            query += " " + std::to_string(slot_val.as_float());
            break;
          case CLIPS::TYPE_INTEGER:
            query += " " + std::to_string(slot_val.as_integer());
            break;
          case CLIPS::TYPE_SYMBOL:
          case CLIPS::TYPE_STRING:
          default:
            // This probably breaks for some other types.
            query += " " + slot_val.as_string();
            break;
        }
      }
      query += "))";
      return has_fact(fact_set_template, query);
    }
};
