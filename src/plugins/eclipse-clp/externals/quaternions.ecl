
/***************************************************************************
 *  blackboard.ecl - Access the blackboard from ECLiPSe
 *
 *  Created: Tue May 20 16:42:47 2014
 *  Copyright  2014 Gesche Gierse
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

%% module definition
:- module(quaternions).

:- export quat_get_yaw/2.

%% definition of external predicates
:- external(quat_get_yaw/2, p_get_yaw).
