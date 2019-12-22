/***************************************************************************
 *  stn.cpp - stn-generator
 *
 *  Created: Sun Dec 22 22:41 2019
 *  Copyright  Nils Adermann <naderman@naderman.de>, Daniel Habering
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

#include "pddl_grammar.h"

using namespace pddl_parser;
namespace phoenix = boost::phoenix;

void
pddl_parser::Grammar::insert_typed_name_entities(TypedList &                     entities,
                                                 const std::vector<std::string> &names,
                                                 const std::string &             type)
{
	std::for_each(names.begin(),
	              names.end(),
	              (phoenix::push_back(phoenix::ref(entities),
	                                  phoenix::construct<struct Entity>(phoenix::arg_names::_1,
	                                                                    phoenix::ref(type)))));
}