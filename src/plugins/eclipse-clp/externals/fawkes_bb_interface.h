
/***************************************************************************
 *  fawkes_bb_interface.h - External predicates to access Fawkes interfaces
 *
 *  Created: Wed Jul 15 13:54:15 2009
 *  Copyright  2009  Daniel Beck
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

#ifndef __PLUGINS_ECLIPSE_CLP_EXTERNALS_FAWKES_BB_INTERFACE_H_
#define __PLUGINS_ECLIPSE_CLP_EXTERNALS_FAWKES_BB_INTERFACE_H_


extern "C" int p_test();
extern "C" int p_blackboard_add_interface();
extern "C" int p_read_interface();
extern "C" int p_write_interface();
extern "C" int p_send_message();
extern "C" int p_recv_messages();

#endif /* __PLUGINS_ECLIPSE_CLP_EXTERNALS_FAWKES_BB_INTERFACE_H_ */
