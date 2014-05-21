
/***************************************************************************
 *  quaternion_helper.cpp - External predicates to transform quaternions
 *
 *  Created: Tue May 20 16:07:07 2014
 *  Copyright  2014  Gesche Gierse
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

#include "quaternion_helper.h"


#include <eclipseclass.h>
#include <tf/types.h>
#include <LinearMath/btQuaternion.h>

#include <cstring>
#include <cstdlib>

using namespace fawkes;
/* This class is a wrapper for tf/types.h
 * Currently it only includes one method to transform quaternions to a yaw value.
*/

int
p_get_yaw()
{
    int res;
    double quad[4];
    EC_word list(EC_arg(1));
    EC_word head, tail;
    if (list.is_list(head,tail) != EC_succeed){
      printf("p_get_yaw(): first argument is not a list!\n");
    }
    for (int i=0 ; list.is_list(head,tail) == EC_succeed and i < 4; list=tail, i++)
    {
        res = head.is_double(&quad[i]);
        if (res != EC_succeed){
            printf( "p_get_yaw(): quaternion is not a list of 4 doubles/floats\n" );
            return EC_fail;
        }
    }
    double yaw = fawkes::tf::get_yaw(btQuaternion((float) quad[0], (float) quad[1], (float) quad[2], (float) quad[3]));
    if ( EC_succeed != EC_arg( 2 ).unify( EC_word(yaw) ) )
    {
	    printf( "p_get_yaw(): could not bind return value\n" );
	    return EC_fail;
    }
    return EC_succeed;
}

