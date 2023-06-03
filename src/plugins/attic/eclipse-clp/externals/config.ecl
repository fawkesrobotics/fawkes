
/***************************************************************************
 *  blackboard.ecl - Access the blackboard from ECLiPSe
 *
 *  Created: Wed Mar 09 17:10:54 2011
 *  Copyright  2011  Daniel Beck
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
:- module(config).

:- export get_config/2.

%% definition of external predicates
:- external(get_config/2, p_get_config_value).
