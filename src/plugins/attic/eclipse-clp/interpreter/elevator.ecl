/***************************************************************************
 *  elevator.ecl - Elevator agent example
 *
 *  Created: Thu Nov 22 15:09:34 2012
 *  Copyright  2012  Gesche Gierse
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


/*
 * Elevator example in vanilla golog
 */
:- module(elevator).
:- use_module("../utils/logging").
:- use_module("../utils/tktools").
:- use_module("golog").


:- log_info("Loading elevator agent").

:- dynamic update/1.
:- dynamic terminate/1.


%% event handlers
handle_update(update) :-
        log_debug("Event: UPDATE"),
        date(Date),
        asserta(update(Date)).

handle_terminate(terminate) :-
        log_debug("Event: TERMINATE"),
        asserta(terminate(1)).

%% setup event handlers
:- set_event_handler(update, handle_update/1).
:- set_event_handler(terminate, handle_terminate/1).

/* primitive control actions */
prim_action(open).            % open the doors
prim_action(close).           % close the doors
prim_action(up(_N)).           % go up to N-th floor
prim_action(down(_N)).         % go down to N-th floor
prim_action(turnoff(_N)).      % turn off call button N

/* preconditions for primitive actions */
poss(open,_S).
poss(close,_S).
poss(up(N),S) :- golog:holds(current_floor(M),S), M < N.
poss(down(N),S) :- golog:holds(current_floor(M),S), M > N.
poss(turnoff(N),S) :- golog:holds(on(N),S).

/* SSAs for primitive fluents */
/*
holds(current_floor(M),do(E,S)) :- 
	E = up(M); 
	E = down(M);
	not E = up(N), not E = down(N), holds(current_floor(M), S).

holds(on(M), do(E,S)) :-
	holds(on(M), S), not E = turnoff(M).
*/

/* procedures */
proc(go_floor(N), ?(current_floor(N)) # up(N) # down(N)).
proc(serve(N), [go_floor(N), turnoff(N), open, close]).
proc(serve_a_floor, pi(n, [?(next_floor(n)), serve(n)])).
proc(park, if(current_floor(0), open, [down(0), open])).
proc(control, [while(some(n, on(n)), serve_a_floor), park]).

/*
holds(next_floor(N),S) :- writeln(holds(next_floor(N),S)), holds(on(N),S).
*/

/* initial situation */
/*
holds(on(3),s0) :- write('initial 1').
holds(on(5),s0) :- write('initial 2').
holds(current_floor(4),s0) :- write('initial 3').
*/

:- local holds/2.
holds(current_floor(4),s0).
holds(current_floor(M),doo(E,S)) :- 
	E = up(M); 
	E = down(M);
	not E = up(N), not E = down(N), golog:holds(current_floor(M), S).
holds(on(3),s0).
holds(on(5),s0).
holds(on(M), doo(E,S)) :-
	golog:holds(on(M), S), not E = turnoff(M).
holds(next_floor(N),S) :- /*writeln(holds(next_floor(N),S)),*/ golog:holds(on(N),S).

%the acutal program being performed (called from eclipse_thread.cpp)
run :-
    (
      /*(
      terminate(_),!,
      log_info("Are you here?"),
      log_info("Terminated")
      )      
      ;*/
      ensure_attached,
      log_info("Request: doo(control,s0,S)"), 
      trace(doo(control,s0,S)),
		  log_info("Result: %w",[S]),
      repeat, %will loop infinitly, needed for testing tktool attachment
      fail   
    ).
		

:- log_info("Loading elevator agent done").
