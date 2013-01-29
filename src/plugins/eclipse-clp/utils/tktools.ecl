:- module(tktools).
:- lib(remote_tools).

:- use_module("interfaces"). 
:- use_module("../utils/logging").
:- export(attach_tktools/0).
:- reexport remote_tools.
:- tool(attach_tktools/0, attach_tktools/1).
:- mode connecting(++,++,++).

%:- local struct( data_EclipseDebuggerInterface(port,host) ).
:- writeln("loading tktools").

attach_tktools(Module) :- log_info("Attaching tktools, please enter the following host and port in tktools..."),
  %attach_tools(H/P, block, writeln(H / P)),
  %trace(  connecting(H,P,Module)),
  attach_tools(H/P, block, connecting(H,P,Module)),
  log_info("tktools attached successfully").

connecting(H, P, M) :- writeln(H / P),
                       add_interface("EclipseDebuggerInterface","eclipse_clp_connect", Struct),
                       writeln(Struct),
                       writeln(M).
                       %assert(local(Struct)).
              %call(write_interface("eclipse_clp_connect",  data_EclipseDebuggerInterface{port:P, host:H}))@M.

