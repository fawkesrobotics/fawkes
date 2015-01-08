
/***************************************************************************
 *  mod_utils.cpp -  OpenPRS general utils module
 *
 *  Created: Fri Aug 29 19:37:19 2014
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
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

#include "mod_utils.h"
#include <default-hook.h>
#include <oprs-rerror_f-pub.h>

/// @cond external
extern "C" {
  typedef struct op_structure Op_Structure;

  typedef enum {IS_ACTIVE, IS_SLEEPING, IS_SUSP_ACTIVE, IS_SUSP_SLEEPING} Intention_Status;
  typedef Slist *Thread_Intention_Block_List;
  typedef Slist *Condition_List;
  struct intention {
     Fact * fact;
     Goal * goal;
     Thread_Intention_Block_List fils;
     Thread_Intention_Block_List active_tibs;
     Op_Instance *top_op;
     short priority;
     Intention_Status status;
     Symbol id;
     Thread_Intention_Block *critical_section;
     PDate creation;
     Condition_List activation_conditions_list;
     Sprinter *failed_goal_sprinter;
     OPRS_LIST failed_goal_stack;
  };
}
/// @endcond

#include <op-instance_f-pub.h>
#include <op-structure_f-pub.h>
#include <lisp-list_f-pub.h>

extern "C"
Term *
func_op_name(TermList terms)
{
  Term *op_t;
  Op_Instance *opi;

  op_t  = (Term *)get_list_pos(terms, 1);
  if (! op_t ) {
    report_fatal_external_error((char *)"Cannot retrieve OP instance term");
  }
  if (op_t->type != TT_OP_INSTANCE) {
    report_fatal_external_error((char *)"Term is not of type OP_INSTANCE");
  }

  opi = (Op_Instance *)(op_t->u.opi);
  Op_Structure *op_s = op_instance_op(opi);
  if (! op_s) {
    report_fatal_external_error((char *)"Failed to get OP structure from OP instance");
  }

  return build_id(op_name(op_s));
}


extern "C"
Term *
func_op_names(TermList terms)
{
  Term *ops_t;

  ops_t  = (Term *)get_list_pos(terms, 1);
  if (! ops_t ) {
    report_fatal_external_error((char *)"Cannot retrieve OP instance term");
  }
  if (ops_t->type != LISP_LIST) {
    report_fatal_external_error((char *)"Term is not of type LISP_LIST");
  }

  TermList name_list = sl_make_slist();

  for (L_List p_l = ops_t->u.l_list; p_l; p_l = l_cdr(p_l)) {
    Term *t = l_car(p_l);
    if (t->type == TT_INTENTION) {
      Op_Instance *opi = (Op_Instance *)(t->u.in->top_op);
      Op_Structure *op_s = op_instance_op(opi);
      name_list = build_term_list(name_list, build_id(op_name(op_s)));
    } else if (t->type == TT_OP_INSTANCE) {
      Op_Instance *opi = (Op_Instance *)(t->u.opi);
      Op_Structure *op_s = op_instance_op(opi);
      if (! op_s) {
        name_list = build_term_list(name_list, build_id(declare_atom("NOT-AN-OP")));
      } else {
        name_list = build_term_list(name_list, build_id(op_name(op_s)));
      }
    } else {
      name_list = build_term_list(name_list, build_id(declare_atom("NOT-AN-OP-INSTANCE")));
    }
  }

  return build_term_l_list_from_c_list(name_list);
}


/** Entry function for the OpenPRS module. */
extern "C"
void init()
{
  printf("*** LOADING mod_utils\n");
  make_and_declare_eval_funct("op-name", func_op_name, 1);
  make_and_declare_eval_funct("op-names", func_op_names, 1);

  const char *gdb_delay_env = getenv("FAWKES_OPRS_GDB_DELAY");
  if (gdb_delay_env && strcmp(gdb_delay_env, "true") == 0) {
    fprintf(stderr,
            "\n============================================================================\n\n"
            "GDB delay enabled. Waiting for 10 seconds. Connect with GDB using:\n\n"
            "gdb -p %i\n\n"
            "============================================================================\n\n",
            getpid());
    sleep(10);
  }
}
