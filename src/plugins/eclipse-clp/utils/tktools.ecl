/***************************************************************************
 *  tktools.ecl - A module needed to connect with tktools via eclipsebugger 
 *  for remote debugging
 *
 *  Created: Tue Jan 29 12:44:54 2013
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


:- module(tktools).
:- lib(remote_tools).

:- use_module("../externals/blackboard").
:- use_module("../utils/logging").
:- export(attach_tktools/0).
:- export(ensure_attached/0).
:- reexport remote_tools.
:- log_info("Loading tktools").

:- local initialization(init).
:- local(finalization(fin)).

init :- bb_open_interface(w,"EclipseDebuggerInterface","eclipse_clp_connect").
fin :- bb_close_interface("EclipseDebuggerInterface::eclipse_clp_connect"),
	log_info("Closing eclipse_clp_connect").

%% event handlers
handle_check_debug_msg(check_debug_msg) :-
  bb_recv_messages("EclipseDebuggerInterface::eclipse_clp_connect", List),
  eval_list(List). 

%% set event handlers
:- set_event_handler(check_debug_msg, handle_check_debug_msg/1).


%check for ConnectionMessage
eval_list([]).
eval_list([Head|Tail]) :- eval_msg(Head), eval_list(Tail).

eval_msg(["ConnectionMessage"|_]) :- attach_tktools.
eval_msg(_). % a fail in a event handle would lead to bugs, so just ignore everything which is not a connection message (shouldn't happen anyhow)



attach_tktools :- log_debug("Attaching tktools, using the following host and port..."),
  attach_tools(H/P, block, connecting(H,P)),
  log_debug("tktools attached successfully").


connecting(H, P) :- log_debug( "%s / %d", [H, P]),
                    bb_set("EclipseDebuggerInterface::eclipse_clp_connect", "port", P),
                    bb_set("EclipseDebuggerInterface::eclipse_clp_connect", "host", H),
                    bb_write_interface("EclipseDebuggerInterface::eclipse_clp_connect").

%after this succeeds, trace/1 and other debug predicates can be called
ensure_attached :- log_info("Waiting for eclipsedebugger to connect, please start eclipsedebugger"), ensure_attached_.
ensure_attached_ :- (attached(_), log_info("eclipsedebugger successfully connected")) ; ( sleep(0.5), ensure_attached_ ).

:- log_info( "Loading tktools done" ).
