/***************************************************************************
 *  skillexec.ecl - Skillexec agent example
 *
 *  Created: Fri Mar 22 16:01:37 2013
 *  Copyright  2013  Gesche Gierse
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


:- module(skillexec).
:- use_module("../externals/blackboard").
:- use_module("../utils/logging").
:- use_module("../utils/tktools").

:- local initialization(init).

:- log_info("Loading skillexec agent").

:- dynamic update/1.
:- dynamic terminate/1.


%% event handlers
handle_update(update) :-
        log_debug("Event: UPDATE"),
        date(Date),
        asserta(update(Date)).

handle_terminate(terminate) :-
        log_debug("Event: TERMINATE"),
        bb_send_message("Skiller", "ReleaseControlMessage", []),
        asserta(terminate(1)).

handle_check_interfaces_msg(check_interfaces_msg) :-
        log_debug("Event: INTERFACES"),
        bb_read_interfaces.

%% setup event handlers
:- set_event_handler(update, handle_update/1).
:- set_event_handler(terminate, handle_terminate/1).
:- set_event_handler(check_interfaces_msg, handle_check_interfaces_msg/1).

init :- bb_ensure_connected,!,
        bb_open_interface(r,"SkillerInterface","Skiller"),
        bb_read_interfaces,
        bb_read_interface("Skiller","exclusive_controller",0),
        bb_send_message("Skiller", "AcquireControlMessage", []),
        log_debug("skillexec.ecl: acquired control.").

%the acutal program being performed (called from eclipse_thread.cpp)
run :-
    (
      log_debug("Writing message to Skiller"),
      bb_send_message("Skiller", "ExecSkillContinuousMessage", [["skill_string", "say{text=\"Hello world\"}"]]),
      repeat, %will loop infinitly, needed for testing tktool attachment
      fail
    ).


:- log_info("Loading skillexec agent done").
