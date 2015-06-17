/***************************************************************************
 *  simulate_execution.ecl - Simulate action execution
 *
 *  Created: Thu Sep 11 11:06:42 2014
 *  Copyright  2014  Till Hofmann
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

/*  This enables execution simulation for an agent. If simulate_execution is
 *  set, all actions will be simulated by logging the executed action.
 *  Furthermore, any "real" action execution is prevented.
 *
 *  For sensing actions, you can provide the sensing result in the form
 *      sensing_result(simulatedAction(arg), value)
 *  If no sensing result is provided, you will be asked for the sensing result
 *  when the sensing action is executed.
 *
 *  If ask_for_exogenous_events is set, you will be asked for exogenous events
 *  after each action execution.
 */

:- multifile(execute/2).

:- dynamic(simulate_execution/0).
:- dynamic(ask_for_exogenous_events/0).

wait :- sleep(0.1).

execute(Action, Sr) :-
  simulate_execution,
  sensing_result(Action, Sr), !,
  log_info("executing %w, result: %w", [Action, Sr]),
  wait.
execute(Action, Sr) :-
  simulate_execution, senses(Action, _), !,
  ask_execute(Action, Sr).
execute(Action, _) :-
  simulate_execution, !,
  log_info("executing %w", [Action]),
  wait.

% do not allow normal action execution.
execute(Action,_) :- 
  simulate_execution, !, 
  log_error("Simulation of action %w failed", [Action]),
  fail.

exog_occurs(Action) :- ask_for_exogenous_events, ask_exog_occurs(Action).
