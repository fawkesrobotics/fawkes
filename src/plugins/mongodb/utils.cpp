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
