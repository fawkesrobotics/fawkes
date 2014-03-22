/***************************************************************************
 *  Reserved_node_constraint.cpp
 *
 *  Created: Sun Mar 02 10:47:35 2014
 *  Copyright  2014  Sebastian Reuter
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

#include "reserved_node_constraint.h"

namespace fawkes{


ReservedNodeConstraint::ReservedNodeConstraint(Logger *logger, std::string name) :
		AbstractNodeConstraint( logger, name)	{

	reserveLogger = logger;
	constraintName = name;
}

ReservedNodeConstraint::~ReservedNodeConstraint(){
}


}
