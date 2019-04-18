/***************************************************************************
 *  utils.cpp - Helper functions for mongodb
 *
 *  Created: Thu 11 Apr 2019 17:58:59 CEST 17:58
 *  Copyright  2019  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include "utils.h"

#include <core/exception.h>

#include <bsoncxx/exception/exception.hpp>
#include <bsoncxx/types.hpp>

using namespace bsoncxx;
using namespace std;

document::element
get_dotted_field(const document::view &doc, const string &key)
{
	bsoncxx::document::view subdoc{doc};
	std::string             subkey = key;
	size_t                  pos;
	while ((pos = subkey.find(".")) != std::string::npos) {
		subdoc = subdoc[subkey.substr(0, pos)].get_document().view();
		subkey = subkey.substr(pos + 1);
	}
	return subdoc[subkey];
}

/**
 * Split a string of the form "<dbname>.<collname>" into a pair (<dbname>, <collname>).
 * @param dbcollection A string of the form "<dbname>.<collname>"
 * @return A pair consisting of the database name and the collection name
 */
std::pair<std::string, std::string>
split_db_collection_string(const std::string &dbcollection)
{
	size_t point_pos = dbcollection.find(".");
	if (point_pos == dbcollection.npos) {
		throw fawkes::Exception(
		  "Improper database collection string: '%s', expected string of format '<dbname>.<collname>'");
	}
	return make_pair(dbcollection.substr(0, point_pos),
	                 dbcollection.substr(point_pos + 1, std::string::npos));
}

/** Check if a mongodb command was successful.
 *  @param reply The reply to the command from the server
 *  @return true if the command was successful
 */
bool
check_mongodb_ok(const bsoncxx::document::view &reply)
{
	try {
		if (!reply["ok"]) {
			return false;
		}
		return reply["ok"].get_double() > 0.5;
	} catch (bsoncxx::exception &e) {
		return false;
	}
}
