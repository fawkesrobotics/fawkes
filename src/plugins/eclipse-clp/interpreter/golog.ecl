/***************************************************************************
 *  golog.ecl - Golog language example
 *
 *  Created: Thu Nov 22 15:10:22 2012
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


:- module(golog).

%:- log_info("Loading golog interpreter").

:- export doo/3.
:- tool(doo/3, doo/4).
:- export holds/2.
:- tool(holds/2, holds/3).
:- export op(950, xfy, #).
:- local op(950, xfy, #).


doo([],S,S,_).

/* sequence */
doo([H|T],S,S1,M) :- 
	doo(H,S,S2,M), doo(T,S2,S1,M).

/* test action */
doo(?(P),S,S,M) :- 
	holds(P,S,M).

/* non-deterministic choice */
doo(E1 # E2,S,S1,M) :-
	doo(E1,S,S1,M); doo(E2,S,S1,M).

/* if-then-else */
doo(if(P,E1,E2),S,S1,M) :- 
	doo([?(P),E1] # [?(neg(P)),E2],S,S1,M).

/* star operator */
doo(star(E),S,S1,M) :- 
	doo([] # [E,star(E)],S,S1,M).

/* while */
doo(while(P,E),S,S1,M) :- 
	doo([star([?(P),E]),?(neg(P))],S,S1,M).

/* non-deterministic choice of arguments */
doo(pi(V,E),S,S1,M) :-
	%writeln(pi(V,E)),
	sub(V,_,E,E1),
	%writeln(ok_pi(E1)),
	doo(E1,S,S1,M).

/* procedure call */
doo(E,S,S1,M) :-
	call(proc(E,E1))@M, /*writeln('proc jojo'(E)),*/ doo(E1,S,S1,M).

/* primitive action call */
doo(E,S,doo(E,S),M) :- 
	call(prim_action(E))@M, call(poss(E,S))@M.

sub(_X1,_X2,T1,T2) :- var(T1), T2=T1.
sub(X1,X2,T1,T2) :- not var(T1), T1 = X1, T2 = X2.
sub(X1,X2,T1,T2) :- not T1 = X1, T1 =..[F|L1],sub_list(X1,X2,L1,L2), T2 =..[F|L2].

sub_list(_X1,_X2,[],[]).
sub_list(X1,X2,[T1|L1],[T2|L2]) :- sub(X1,X2,T1,T2), sub_list(X1,X2,L1,L2).


holds(X,S,Module) :- call(holds(X,S))@Module.

holds(and(P1,P2),S,Module) :- holds(P1,S,Module), holds(P2,S,Module).
holds(or(P1,P2),S,Module) :- ( holds(P1,S,Module); holds(P2,S,Module) ).
holds(neg(P),S,Module) :- not holds(P,S,Module).
holds(some(V,P),S,Module) :- sub(V,_,P,P1), holds(P1,S,Module).


%:- log_info("Loading golog interpreter done").
