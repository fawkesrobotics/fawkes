:- module(tktools).
:- lib(remote_tools).

:- use_module("../externals/blackboard"). 
:- use_module("../utils/logging").
:- export(attach_tktools/0).
:- reexport remote_tools.

:- log_info("Loading tktools").

:- local initialization(init).

init :- bb_connect("localhost"),
        bb_open_interface(w,"EclipseDebuggerInterface","eclipse_clp_connect").


%% event handlers
handle_check_debug_msg(check_debug_msg) :- bb_read_interfaces,
                               bb_recv_messages("eclipse_clp_connect", List),
                               eval_list(List). 
%% set event handlers
:- set_event_handler(check_debug_msg, handle_check_debug_msg/1).


%check for ConnectionMessage
eval_list([]).
eval_list([Head|Tail]) :- eval_msg(Head), eval_list(Tail).

eval_msg(["ConnectionMessage"|_]) :- attach_tktools.
eval_msg(_). % a fail in a event handle would lead to bugs, so just ignore everything which is not a connection message (shouldn't happen anyhow)



attach_tktools :- log_debug("Attaching tktools, please enter the following host and port in tktools..."),
  attach_tools(H/P, block, connecting(H,P)),
  log_debug("tktools attached successfully").


connecting(H, P) :- log_debug( "%s / %d", [H, P]),
                    bb_write_interface("eclipse_clp_connect", "port", P),
                    bb_write_interface("eclipse_clp_connect", "host", H),
                    bb_write_interfaces.

:- log_info( "Loading dummy interpreter done" ).
