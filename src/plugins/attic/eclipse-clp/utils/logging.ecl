
/***************************************************************************
 *  logging.ecl - Some utility predicates for using the fawkes logging.
 *
 *  Created: Wed Jul 22 13:43:47 2009
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

:- module(logging).
:- export log_info/2.
:- export log_info/1.
:- export log_warn/2.
:- export log_warn/1.
:- export log_error/2.
:- export log_error/1.
:- export log_debug/2.
:- export log_debug/1.
:- external(log/2, p_log).

:- log(ll_info, "Loading logging utils").

log_debug(S) :-
        log(ll_debug, S).

log_info(S) :-
        log(ll_info, S).

log_warn(S) :-
        log(ll_warn, S).

log_error(S) :-
        log(ll_error, S).

log(LogLevel, Format, ArgList) :-
        sprintf(S, Format, ArgList),
        log(LogLevel, S).

log_debug(Format, ArgList) :-
        sprintf(S, Format, ArgList),
        log_debug(S).

log_info(Format, ArgList) :-
        sprintf(S, Format, ArgList),
        log_info(S).

log_warn(Format, ArgList) :-
        sprintf(S, Format, ArgList),
        log_warn(S).

log_error(Format, ArgList) :-
        sprintf(S, Format, ArgList),
        log_error(S).

:- log_info("Loading logging utils done").
