:- module(interfaces).

:- export(whatsin/0).
:- export(add_interface/3).
%:- external(blackboard_add_interface/3, p_blackboard_add_interface).
%:- external(blackboard_add_interface/3, p_test).
:- mode add_interface(++,++, -).
:- writeln("Loading interfaces utils").


whatsin :- current_struct(data_EclipseDebuggerInterface,X), writeln(X).


:- writeln("A").
%getStruct(Host, Port, data_EclipseDebuggerInterface{port:Port, host:Host}).


add_interface(Type, Id, Struct) :- blackboard_add_interface(Type, Id, Struct).


%add_interface(Type, Id) :- blackboard_add_interface(Type, Id, Struct), assert(export Struct).

:- writeln("B").
