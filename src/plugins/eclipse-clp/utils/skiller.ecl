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

:- use_module(filepath).

?- locate_module("blackboard", F), use_module(F).
?- locate_module("logging", F), use_module(F).

:- export(exec_skill/2).
:- export(exec_skill2/1).
:- export(exec_skill_id/3).
:- export(exec_skill_id2/2).
:- export(wait_for_skiller/0).
:- export(decide_on_sensing/1).
:- export(is_final/1).
:- export(is_failed/1).
:- export(is_running/1).
:- export(not_running/1).


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

success :- bb_get("SkillerInterface::Skiller", "status", Status), is_final(Status).
failed :- bb_get("SkillerInterface::Skiller", "status", Status), is_failed(Status).

%% wait until skiller is done
wait_for_skiller :-
    bb_read_interfaces,
    bb_get("SkillerInterface::Skiller", "status", Status),
    is_running(Status), sleep(0.1),
    wait_for_skiller.
wait_for_skiller :-
    bb_get("SkillerInterface::Skiller", "status", Status),
    not_running(Status).

%% wait until the skiller started to process the message with the given MsgID
%% use with caution: If the message has been processed and the skiller has
%% already started to process a new message, this will end in a deadlock.
wait_until_processed(MsgID) :-
    bb_read_interfaces,
    bb_get("SkillerInterface::Skiller", "msgid", CurrentMsgID),
    MsgID \= CurrentMsgID,
    sleep(0.1),
    wait_until_processed(MsgID).
wait_until_processed(MsgID) :-
    bb_read_interfaces,
    bb_get("SkillerInterface::Skiller", "msgid", CurrentMsgID),
    MsgID == CurrentMsgID.


%% auxiliary predicates to execute skills
exec_skill(Skill, Arguments) :-
    append_strings(Skill, "{", Str1),
    append_strings(Str1, Arguments, Str2),
    append_strings(Str2, "}", Skillmsg),
    exec_skill2(Skillmsg).

exec_skill2(Skillmsg) :-
    log_info("Executing skill '%w'", Skillmsg),
    bb_send_message("SkillerInterface::Skiller", "ExecSkillMessage", [["skill_string", Skillmsg]], MsgID),
    log_info("Sending message with ID %w", [MsgID]),
    wait_until_processed(MsgID),
    log_info("Sent Skiller Message").

% returns MsgID of the skill execution
exec_skill_id(Skill, Arguments, MsgID) :-
    append_strings(Skill, "{", Str1),
    append_strings(Str1, Arguments, Str2),
    append_strings(Str2, "}", Skillmsg),
    bb_send_message("SkillerInterface::Skiller", "ExecSkillMessage", [["skill_string", Skillmsg]], MsgID),
    wait_until_processed(MsgID).

exec_skill_id2(Skillmsg, MsgID) :-
    bb_send_message("SkillerInterface::Skiller", "ExecSkillMessage", [["skill_string", Skillmsg]], MsgID),
    wait_until_processed(MsgID).

