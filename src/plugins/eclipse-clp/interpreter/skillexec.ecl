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

%% setup event handlers
:- set_event_handler(update, handle_update/1).
:- set_event_handler(terminate, handle_terminate/1).

init :- bb_ensure_connected,!,
        bb_open_interface(r,"SkillerInterface","Skiller"),
        bb_open_interface(w,"TestInterface","eclipse_clp_skillexec"),
        bb_read_interfaces,
        bb_read_interface("Skiller","exclusive_controller",0),
        bb_send_message("Skiller", "AcquireControlMessage", []),
        log_debug("skillexec.ecl: acquired control.").

%the acutal program being performed (called from eclipse_thread.cpp)
cycle :-
    (
      bb_read_interfaces,
      check_for_msg,
      run_agent_once,
      bb_write_interfaces
    ).

% this predicate is called at the start of the think-hook
% if an agent is writer to some interface, it should check for new messages here
check_for_msg :-
        bb_recv_messages("eclipse_clp_skillexec", List),
        eval_list(List).

% check for SetTestStringMessage - this is only neccessary to use the skilltester tool
eval_list([]).
eval_list([Head|Tail]) :- eval_msg(Head), eval_list(Tail).

eval_msg(["SetTestStringMessage"|[[["test_string"|[Skill]]]]]) :- exec_skill2(Skill).
eval_msg(_). % check only. If no msg is there, still succeed.


run_agent_once :-
      log_debug("Writing message to Skiller"),
      exec_skill("say","text=\"In Cycle\"").

exec_skill(Skill, Arguments) :-
    append_strings(Skill, "{", Str1),
    append_strings(Str1, Arguments, Str2),
    append_strings(Str2, "}", Str3),
    bb_send_message("Skiller", "ExecSkillContinuousMessage", [["skill_string", Str3]]).

exec_skill2(Skillmsg) :- bb_send_message("Skiller", "ExecSkillContinuousMessage", [["skill_string", Skillmsg]]).


:- log_info("Loading skillexec agent done").
