%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FILE: Elevator-Vanilla/main2_ecl.pl
%
%  AUTHOR : Sebastian Sardina (2002)
%  EMAIL  : ssardina@cs.toronto.edu
%  WWW    : www.cs.toronto.edu/~ssardina www.cs.toronto.edu/cogrobo
%  TYPE   : system dependent code
%  TESTED : ECLIPSE Version 5.10 44, Sun Jan 14 02:06 2007
%
%  This is example1 for the first IndiGolog code written by H. Levesque
%
% Written for ECLIPSE Prolog http://eclipse.crosscoreop.com/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                             June 15, 2000
%
% This software was developed by the Cognitive Robotics Group under the
% direction of Hector Levesque and Ray Reiter.
%
%        Do not distribute without permission.
%        Include this notice in any copy made.
%
%
%         Copyright (c) 2000 by The University of Toronto,
%                        Toronto, Ontario, Canada.
%
%                          All Rights Reserved
%
% Permission to use, copy, and modify, this software and its
% documentation for non-commercial research purpose is hereby granted
% without fee, provided that the above copyright notice appears in all
% copies and that both the copyright notice and this permission notice
% appear in supporting documentation, and that the name of The University
% of Toronto not be used in advertising or publicity pertaining to
% distribution of the software without specific, written prior
% permission.  The University of Toronto makes no representations about
% the suitability of this software for any purpose.  It is provided "as
% is" without express or implied warranty.
% 
% THE UNIVERSITY OF TORONTO DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS
% SOFTWARE, INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND
% FITNESS, IN NO EVENT SHALL THE UNIVERSITY OF TORONTO BE LIABLE FOR ANY
% SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER
% RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF
% CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
% CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
% 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONSULT NECESSARY FILES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
:- module(indi_elevator).
% 1 - Consult the top-level interpreter
:- use_module("interrupts").
:- use_module("indigolog_vanilla").
:- use_module("../utils/logging.ecl").
:- use_module("../utils/tktools.ecl").
:- use_module("../utils/check_indigolog.ecl").

% used to meassure time
:- lib(util).

% dynamic is needed for basic_check from check_indigolog, afterwards they
% are re-compiled to static.
:- dynamic execute/2.
:- dynamic prim_action/1.
:- dynamic poss/2.
:- dynamic prim_fluent/1.
:- dynamic initially/2.
:- dynamic senses/2.
:- dynamic causes_val/4.
:- dynamic proc/2.

%% event handlers
handle_update(update).
:- set_event_handler(update, handle_update/1).

% Serve each floor whose call button is on initially, then park the elevator.
% run: ?- indigolog(smart_control)  use search
% run: ?- indigolog(dumb_control)   the same but without search (fails)
%
% No user input is required.

% Interface to the outside world via read and write 
execute(A,Sr) :- ask_execute(A,Sr).
exog_occurs(_) :- fail.

fl(N) :- N=1; N=2; N=3; N=4; N=5; N=6.    % the 6 elevator floors

% Actions 
prim_action(down).              % elevator down one floor
prim_action(up).                % elevator up one floor
prim_action(off(N)) :- fl(N).   % turn off call button on floor n
prim_action(open).              % open elevator door
prim_action(close).             % close elevator door

% Fluents 
prim_fluent(floor).             % the floor the elevator is on (1 to 6)
prim_fluent(light(N)) :- fl(N). % call button of floor n (on or off)

% Causal laws 
causes_val(up,   floor, N, N is floor+1).
causes_val(down, floor, N, N is floor-1).
causes_val(off(N), light(N), off, true).  % Note: nothing turns a light on

senses(_,_) :- fail.

% Preconditions of prim actions
poss(down,    neg(floor=1)).
poss(up,      neg(floor=6)).
poss(off(N),  and(floor=N,light(N)=on)).
poss(open, true).
poss(close, true).

% Initial state: elevator is at floor 3, and lights 2 and 5 are on
initially(floor,3).
initially(light(1), off).
initially(light(2), on).
initially(light(3), off).
initially(light(4), off).
initially(light(5), on).
initially(light(6), off).

% Definitions of complex conditions
proc(below_floor(N), floor<N).
proc(above_floor(N), floor>N).

% Definitions of complex actions
proc(go_floor(N), while(neg(floor=N), if(below_floor(N),up,down))).
proc(serve_floor(N), [ go_floor(N), open, close, off(N) ]).

proc(handle_reqs(Max),      % handle all elevator reqs in Max steps
    ndet(  [?(and(neg(some(n,light(n)=on)),Max>=floor-1)), go_floor(1), open],
            pi(n, pi(m, [ ?(and(light(n)=on, m is Max-abs(floor-n))),
                          ?(m > 0),
                          serve_floor(n),
                          handle_reqs(m) ] )))).

proc(minimize_motion(Max),  % iterative deepening search 
    ndet( handle_reqs(Max), pi(m, [?(m is Max+1), minimize_motion(m)]))).

proc(dumb_control, minimize_motion(0) ).           % always fails 
proc(smart_control, search(minimize_motion(0)) ).  % eventually succeeds


%%%%%%%%% Main cycle from embedded eclipse %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

run :-	use_interrupts, basic_check, writeln("elevator go"), time(indigolog(smart_control)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% EOF: Elevator-Vanilla/main2_swi.pl
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
