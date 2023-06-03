:- module(interrupts).

:- export use_interrupts/0.
:- tool(use_interrupts/0, use_interrupts/1).

%:- export initialization((interrupts: use_interrupts)).



:- writeln("Loading interrupts").

       /* (d) -  INTERRUPTS */
use_interrupts(M) :-
    writeln("use interrupts"),
    writeln(M),
    call(assert(prim_action(start_interrupts)))@M,
    call(assert(prim_action(start_interrupts)))@M,
    call(assert(prim_action(stop_interrupts)))@M,
    call(assert(prim_fluent(interrupts)))@M,
    call(assert(causes_val(start_interrupts, interrupts, running, true)))@M,
    call(assert(causes_val(stop_interrupts, interrupts, stopped, true)))@M,
    call(assert(poss(start_interrupts, true)))@M,
    call(assert(poss(stop_interrupts,  true)))@M,
    /* version with variable */
    call(assert(proc(interrupt(V,Trigger,Body),
        while(interrupts=running, pi(V,if(Trigger,Body,?(neg(true))))))))@M,
    /* version without variable */
    call(assert(proc(interrupt(Trigger,Body),
        while(interrupts=running, if(Trigger,Body,?(neg(true)))))))@M,
    call(assert((proc(prioritized_interrupts(L),[start_interrupts,E]) :- expand_interrupts(L,E))))@M,
    call(assert(expand_interrupts([],stop_interrupts)))@M,
    call(assert((expand_interrupts([X|L],pconc(X,E)) :- expand_interrupts(L,E))))@M,
    writeln("asserted all").

