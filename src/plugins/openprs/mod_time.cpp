
/***************************************************************************
 *  mod_skiller.cpp -  OpenPRS skiller module
 *
 *  Created: Fri Aug 22 14:32:01 2014
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

#include <plugins/openprs/mod_utils.h>

#include <utils/time/time.h>

#include <default-hook.h>
#include <oprs-rerror_f-pub.h>

extern "C"
PBoolean
pred_time_lt(TermList terms)
{
  Term *t1_sec, *t1_usec, *t2_sec, *t2_usec;
  t1_sec  = (Term *)get_list_pos(terms, 1);
  t1_usec = (Term *)get_list_pos(terms, 2);
  t2_sec  = (Term *)get_list_pos(terms, 3);
  t2_usec = (Term *)get_list_pos(terms, 4);

  long long int t1_sec_val, t1_usec_val, t2_sec_val, t2_usec_val;
  if (t1_sec->type == LONG_LONG) {
    t1_sec_val = t1_sec->u.llintval;
  } else if (t1_sec->type == INTEGER) {
    t1_sec_val = t1_sec->u.intval;
  } else {
    fprintf(stderr, "time-lt: t1_sec neither of type integer nor long long\n");
    return false;
  }
  if (t1_usec->type == LONG_LONG) {
    t1_usec_val = t1_usec->u.llintval;
  } else if (t1_usec->type == INTEGER) {
    t1_usec_val = t1_usec->u.intval;
  } else {
    fprintf(stderr, "time-lt: t1_usec neither of type integer nor long long\n");
    return false;
  }
  if (t2_sec->type == LONG_LONG) {
    t2_sec_val = t2_sec->u.llintval;
  } else if (t2_sec->type == INTEGER) {
    t2_sec_val = t2_sec->u.intval;
  } else {
    fprintf(stderr, "time-lt: t2_sec neither of type integer nor long long\n");
    return false;
  }
  if (t2_usec->type == LONG_LONG) {
    t2_usec_val = t2_usec->u.llintval;
  } else if (t2_usec->type == INTEGER) {
    t2_usec_val = t2_usec->u.intval;
  } else {
    fprintf(stderr, "time-lt: t2_usec neither of type integer nor long long\n");
    return false;
  }

  return ((t1_sec_val < t2_sec_val) || (t1_sec_val == t2_sec_val && t1_usec_val < t2_usec_val));
}


extern "C"
PBoolean
pred_time_eq(TermList terms)
{
  Term *t1_sec, *t1_usec, *t2_sec, *t2_usec;
  t1_sec  = (Term *)get_list_pos(terms, 1);
  t1_usec = (Term *)get_list_pos(terms, 2);
  t2_sec  = (Term *)get_list_pos(terms, 3);
  t2_usec = (Term *)get_list_pos(terms, 4);

  if (t1_sec->type != LONG_LONG || t1_usec->type != LONG_LONG ||
      t2_sec->type != LONG_LONG || t2_usec->type != LONG_LONG)
  {
    fprintf(stderr, "time-eq: time values not (all) of type integer (types %i %i %i %i)\n",
    	    t1_sec->type, t1_usec->type, t2_sec->type, t2_usec->type);
    return FALSE;
  }

  //printf("time-eq: %i %i == %i %i? %s\n", t1_sec->u.intval, t1_usec->u.intval, t2_sec->u.intval, t2_usec->u.intval,
  //	 (t1_sec->u.intval == t2_sec->u.intval && t1_usec->u.intval != t2_usec->u.intval) ? "YES" : "NO");

  if (t1_sec->u.intval == t2_sec->u.intval && t1_usec->u.intval == t2_usec->u.intval)
  {
    return TRUE;
  } else {
    return FALSE;
  }
}


extern "C"
PBoolean
pred_time_neq(TermList terms)
{
  Term *t1_sec, *t1_usec, *t2_sec, *t2_usec;
  t1_sec  = (Term *)get_list_pos(terms, 1);
  t1_usec = (Term *)get_list_pos(terms, 2);
  t2_sec  = (Term *)get_list_pos(terms, 3);
  t2_usec = (Term *)get_list_pos(terms, 4);

  if (t1_sec->type != LONG_LONG || t1_usec->type != LONG_LONG ||
      t2_sec->type != LONG_LONG || t2_usec->type != LONG_LONG)
  {
    fprintf(stderr, "time-neq: time values not (all) of type integer (types %i %i %i %i)\n",
    	    t1_sec->type, t1_usec->type, t2_sec->type, t2_usec->type);
    return FALSE;
  }

  /*
  printf("time-neq: %i %i < %i %i? %s\n", t1_sec->u.intval, t1_usec->u.intval, t2_sec->u.intval, t2_usec->u.intval,
	 (t1_sec->u.intval != t2_sec->u.intval) || (t1_sec->u.intval == t2_sec->u.intval && t1_usec->u.intval != t2_usec->u.intval) ? "YES" : "NO");
  */

  if ((t1_sec->u.intval != t2_sec->u.intval) ||
      (t1_sec->u.intval == t2_sec->u.intval && t1_usec->u.intval != t2_usec->u.intval))
  {
    return TRUE;
  } else {
    return FALSE;
  }
}


extern "C"
PBoolean
pred_timeout(TermList terms)
{
  Term *t1_sec, *t1_usec, *t2_sec, *t2_usec, *interval;
  t1_sec   = (Term *)get_list_pos(terms, 1);
  t1_usec  = (Term *)get_list_pos(terms, 2);
  t2_sec   = (Term *)get_list_pos(terms, 3);
  t2_usec  = (Term *)get_list_pos(terms, 4);
  interval = (Term *)get_list_pos(terms, 5);

  if (t1_sec->type != LONG_LONG || t1_usec->type != LONG_LONG ||
      t2_sec->type != LONG_LONG || t2_usec->type != LONG_LONG ||
      (interval->type != LONG_LONG && interval->type != FLOAT && interval->type != INTEGER))
  {
    fprintf(stderr, "timeout: time values not (all) of type LONG_LONG (types %i %i %i %i)\n",
    	    t1_sec->type, t1_usec->type, t2_sec->type, t2_usec->type);
    return FALSE;
  }

  double compare_val = 0;
  if (interval->type == LONG_LONG) {
    compare_val = interval->u.llintval;
  } else if (interval->type == INTEGER) {
    compare_val = interval->u.intval;
  } else if (interval->type == FLOAT) {
    compare_val = *interval->u.doubleptr;
  }

  return (fawkes::time_diff_sec(t1_sec->u.llintval, t1_usec->u.llintval,
				t2_sec->u.llintval, t2_usec->u.llintval) > compare_val);
}

extern "C"
Term *
action_set_idle_looptime(TermList terms)
{
  Term *t_sec, *t_usec;

  t_sec  = (Term *)get_list_pos(terms, 1);
  t_usec = (Term *)get_list_pos(terms, 2);

  if ((t_sec->type != INTEGER && t_sec->type != LONG_LONG) ||
      (t_usec->type != INTEGER && t_usec->type != LONG_LONG))
  {
    fprintf(stderr, "time-set-looptime: time values not (all) of type "
	    "integer (types %i %i)\n", t_sec->type, t_usec->type);
    ACTION_FAIL();
  }

  if (t_sec->type == INTEGER) {
    main_loop_pool_sec = t_sec->u.intval;
  } else if (t_sec->type == LONG_LONG) {
    main_loop_pool_sec = t_sec->u.llintval;
  }

  if (t_usec->type == INTEGER) {
    main_loop_pool_usec = t_usec->u.intval;
  } else if (t_usec->type == LONG_LONG) {
    main_loop_pool_usec = t_usec->u.llintval;
  }

  printf("Setting idle loop time: %li sec  %li usec\n", main_loop_pool_sec, main_loop_pool_usec);
  ACTION_FINAL();
}


/** Entry function for the OpenPRS module. */
extern "C"
void init()
{
  printf("*** LOADING mod_time\n");
  make_and_declare_eval_pred("time-lt", pred_time_lt, 4, TRUE);
  make_and_declare_eval_pred("time-eq", pred_time_eq, 4, TRUE);
  make_and_declare_eval_pred("time-neq", pred_time_neq, 4, TRUE);
  make_and_declare_eval_pred("timeout", pred_timeout, 5, TRUE);
  make_and_declare_action("time-set-idle-looptime", action_set_idle_looptime, 2);
}
