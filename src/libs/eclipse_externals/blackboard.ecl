
:- load("../../../lib/eclipse_externals.so").

:- external(bb_connect/1, p_connect_to_blackboard).
:- external(bb_disconnect/0, p_disconnect_from_blackboard).
:- external(bb_is_alive/0, p_is_alive).
:- external(bb_open_interface/3, p_open_interface).
:- external(bb_close_interface/1, p_close_interface).
:- external(bb_read_interfaces/0, p_read_interfaces).
:- external(bb_write_interfaces/0, p_write_interfaces).
:- external(bb_read_interface/3, p_read_from_interface).
:- external(bb_write_interface/3, p_write_to_interface).

bb_open_interface_writing(Type, Id) :-
        bb_open_interface(w, Type, Id).

bb_open_interface_reading(Type, Id) :-
        bb_open_interface(r, Type, Id).
