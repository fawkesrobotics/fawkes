
/***************************************************************************
 *  blackboard.h - External predicates to remotely access the Fawkes
 *                 blackboard
 *
 *  Created: Wed Mar 09 16:57:03 2011
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

#ifndef __ECLIPSE_EXTERNALS_BLACKBOARD_H_
#define __ECLIPSE_EXTERNALS_BLACKBOARD_H_

extern "C" int p_connect_to_remote_blackboard();
extern "C" int p_connect_to_eclipse_blackboard();
extern "C" int p_disconnect_from_blackboard();
extern "C" int p_is_alive();
extern "C" int p_is_connected();

extern "C" int p_open_interface();
extern "C" int p_close_interface();

extern "C" int p_has_writer();
extern "C" int p_instance_serial();

extern "C" int p_read_interfaces();
extern "C" int p_write_interfaces();

extern "C" int p_read_from_interface();
extern "C" int p_write_to_interface();

extern "C" int p_send_message();
extern "C" int p_recv_messages();

#endif

