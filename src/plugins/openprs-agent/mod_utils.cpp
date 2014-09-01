
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

extern "C" {
  typedef struct op_structure Op_Structure;
}

#include <op-instance_f-pub.h>
#include <op-structure_f-pub.h>

extern "C"
Term *
func_op_name(TermList terms)
{
  Term *op_t, *res;
  Op_Instance *opi;

  op_t  = (Term *)get_list_pos(terms, 1);
  if (! op_t ) {
    report_fatal_external_error((char *)"Cannot retrieve OP instance term");
  }
  if (! op_t->type == TT_OP_INSTANCE) {
    report_fatal_external_error((char *)"Term is not of type OP_INSTANCE");
  }

  opi = (Op_Instance *)(op_t->u.opi);
  Op_Structure *op_s = op_instance_op(opi);
  if (! op_s) {
    report_fatal_external_error((char *)"Failed to get OP structure from OP instance");
  }
  Symbol name = op_name(op_s);

  res = MAKE_OBJECT(Term);
  res->type = TT_ATOM;
  res->u.id = name;
  return res;
}


/** Entry function for the OpenPRS module. */
extern "C"
void init()
{
  printf("*** LOADING mod_utils\n");
  make_and_declare_eval_funct("op-name", func_op_name, 1);
}
