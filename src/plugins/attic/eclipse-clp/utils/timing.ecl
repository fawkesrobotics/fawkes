/***************************************************************************
 *  timing.ecl - Measures times between tic and toc and writes them to
 *               console (ignoring loglevel).
 *
 *  Created: Mon Mar 24 10:36:49 2015
 *  Copyright  2015  Gesche Gierse
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
:- module(timing).


:- export(tic/0).
:- export(toc/0).

:- dynamic starttime/1.

tic :-
  writeln("Tic"),
  statistics(hr_time, Time),
  assert(starttime(Time)).

toc :-
  statistics(hr_time, Now),
  starttime(Time),
  NeededTime is Now - Time,
  retract(starttime(Time)),
  printf("Eclipse-clp: Time between tic and toc: %w", [NeededTime]), nl.