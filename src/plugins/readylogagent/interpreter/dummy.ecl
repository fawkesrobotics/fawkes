
/***************************************************************************
 *  dummy.ecl - A dummy interpreter that only implements some event
 *  handlers to test the interaction with the readylog agent plugin
 *
 *  Created: Wed Jul 22 11:25:21 2009
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

:- log_info("Loading dummy interpreter").

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

run :-
        shelf_create(summands(0,0), Shelf),
        repeat,
        sleep(0.01),
        process_events(Shelf),
        terminate(_),!,
        shelf_abolish(Shelf),
        log_info("Terminating").

process_events(Shelf) :-
        (
            %% terminate event: do nothing
            terminate(_)
        ;
            %% update event: send message, read inteface
            update(Date),
            %log_debug("Last update happend %w", [Date]),
            retract(update(Date)),

            shelf_get(Shelf, 1, N1),
            shelf_get(Shelf, 2, N2),
            read_result(R),
            log_info("%d + %d = %d", [N1, N2, R]),
            
            random(F1),
            random(F2),
            
            mod(F1, 100, R1),
            mod(F2, 100, R2),
            
            shelf_set(Shelf, 1, R1),
            shelf_set(Shelf, 2, R2),
            send_calculate_msg(R1, R2)
        ).

read_result(R) :-
        read_interface("readylog_test", D),
        arg(result of data_TestInterface, D, R).

send_calculate_msg(N1, N2) :-
        send_message("readylog_test",
                     data_TestInterface_CalculateMessage{summand:N1, addend:N2}).

:- log_info( "Loading dummy interpreter done" ).