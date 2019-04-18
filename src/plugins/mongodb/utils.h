/***************************************************************************
 *  utils.h - Helper functions for mongodb
 *
 *  Created: Thu 11 Apr 2019 17:53:40 CEST 17:53
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

#ifndef _PLUGINS_MONGODB_UTILS_H_
#define _PLUGINS_MONGODB_UTILS_H_

#include <bsoncxx/document/element.hpp>
#include <bsoncxx/document/view.hpp>
#include <string>
#include <utility>

bsoncxx::document::element          get_dotted_field(const bsoncxx::document::view &doc,
                                                     const std::string &            key);
std::pair<std::string, std::string> split_db_collection_string(const std::string &dbcollection);

#endif /* !_PLUGINS_MONGODB_UTILS_H_ */
