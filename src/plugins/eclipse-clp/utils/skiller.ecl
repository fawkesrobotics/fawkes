/***************************************************************************
 *  skiller.ecl - Execute skills from ECLiPSe
 *
 *  Created: Tue Aug 19 13:15:42 2014
 *  Copyright  2014  Gesche Gierse, Till Hofmann
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
:- module(skiller).
:- use_module("../externals/blackboard").
:- use_module("../utils/logging").

:- export(exec_skill/2).
:- export(exec_skill2/1).
:- export(wait_for_skiller/0).
:- export(decide_on_sensing/1).

:- log_info("Loading skiller tools done").

%% auxiliary predicates to handle skiller status
is_running("S_RUNNING").
not_running(Status) :- \+ is_running(Status).
is_final("S_FINAL").
is_failed("S_FAILED").

decide_on_sensing(true) :- success.
decide_on_sensing(false) :- failed.

inverse_decide(false) :- success.
inverse_decide(true) :- failed.

success :- bb_read_interface("Skiller", "status", Status), is_final(Status).
failed :- bb_read_interface("Skiller", "status", Status), is_failed(Status).

%% wait until skiller is done
wait_for_skiller :-
    bb_read_interfaces,
    bb_read_interface("Skiller", "status", Status),
    is_running(Status), sleep(0.1),
    wait_for_skiller.
wait_for_skiller :-
    bb_read_interface("Skiller", "status", Status),
    not_running(Status).

%% wait until skiller is running
%% use with caution:
%% This will only work if the skill is either
%% still running or hasn't started to run.
%% If the skiller is already done, this will end in a deadlock.
wait_for_running :-
    bb_read_interfaces,
    bb_read_interface("Skiller", "status", Status),
    not_running(Status), sleep(0.1),
    wait_for_running.
wait_for_running :-
    bb_read_interface("Skiller", "status", Status),
    is_running(Status).


%% auxiliary predicates to execute skills
exec_skill(Skill, Arguments) :-
    append_strings(Skill, "{", Str1),
    append_strings(Str1, Arguments, Str2),
    append_strings(Str2, "}", Str3),
    log_info("Executing skill '%w'", Str3),
    bb_send_message("Skiller", "ExecSkillContinuousMessage", [["skill_string", Str3]]),
    %wait_for_running,
    sleep(0.5),
    log_info("Sent Skiller Message").

exec_skill2(Skillmsg) :- bb_send_message("Skiller", "ExecSkillContinuousMessage", [["skill_string", Skillmsg]]).

