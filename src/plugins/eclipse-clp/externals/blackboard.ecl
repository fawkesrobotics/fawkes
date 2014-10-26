
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
:- module(blackboard).

:- export bb_connect_remote/1.
:- export bb_disconnect/0.
:- export bb_is_alive/0.
:- export bb_open_interface/3.
:- export bb_open_interface_reading/2.
:- export bb_open_interface_writing/2.
:- export bb_close_interface/1.
:- export bb_has_writer/1.
:- export bb_instance_serial/2.
:- export bb_read_interfaces/0.
:- export bb_write_interfaces/0.
:- export bb_read_interface/1.
:- export bb_write_interface/1.
:- export bb_interface_changed/1.
:- export bb_get/3.
:- export bb_set/3.
:- export bb_send_message/3.
:- export bb_send_message/4.
:- export bb_recv_messages/2.

:- export bb_ensure_connected_remote/1.
:- export bb_ensure_connected/0.

%% definition of external predicates
:- external(bb_connect_remote/1, p_bb_connect_to_remote_blackboard).
:- external(bb_disconnect/0, p_bb_disconnect_from_blackboard).
:- external(bb_is_alive/0, p_bb_is_alive).
:- external(bb_is_connected/0, p_bb_is_connected).
:- external(bb_open_interface/3, p_bb_open_interface).
:- external(bb_close_interface/1, p_bb_close_interface).
:- external(bb_has_writer/1, p_bb_has_writer).
:- external(bb_instance_serial/2, p_bb_instance_serial).
:- external(bb_read_interfaces/0, p_bb_read_interfaces).
:- external(bb_write_interfaces/0, p_bb_write_interfaces).
:- external(bb_read_interface/1, p_bb_read_interface).
:- external(bb_write_interface/1, p_bb_write_interface).
:- external(bb_interface_changed/1, p_bb_interface_changed).
:- external(bb_get/3, p_bb_get).
:- external(bb_set/3, p_bb_set).
:- external(bb_send_message/4, p_bb_send_message).
:- external(bb_recv_messages/2, p_bb_recv_messages).

%% definition of errors
connection_error_handler(_, _) :-
    write("\nNot connected to EclipseAgentThread's Blackboard\n"),
    abort.

:- set_event_handler('Connection error', connection_error_handler/2).

%% shortcuts
bb_open_interface_writing(Type, Id) :-
        bb_open_interface(w, Type, Id).

bb_open_interface_reading(Type, Id) :-
        bb_open_interface(r, Type, Id).

bb_ensure_connected_remote(Host) :- bb_is_connected ; bb_connect_remote(Host).
bb_ensure_connected :- bb_is_connected ; error('Connection error',_).

%% bb_send_message/3 with the MsgID omitted
bb_send_message(Interface, MessageType, Args) :- bb_send_message(Interface, MessageType, Args, _).
