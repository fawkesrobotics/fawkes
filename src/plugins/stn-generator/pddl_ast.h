
/***************************************************************************
 *  pddlast.h
 *
 *  Created: Fri 19 May 2017 14:07:13 CEST
 *  Copyright  2017  Matthias Loebach
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

#ifndef __PLUGINS_PDDL_AST_H
#define __PLUGINS_PDDL_AST_H

#include <boost/spirit/include/qi.hpp>
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/fusion/include/std_pair.hpp>
#include <string>
#include <vector>

namespace pddl_parser {
  namespace qi = boost::spirit::qi;
  namespace ascii = boost::spirit::ascii;

  typedef std::pair<std::string, std::string> pair_type;
  typedef std::vector<pair_type> pairs_type;

  typedef std::vector<std::string> type_list;
  typedef std::pair<type_list, std::string> pair_multi_const;
  typedef std::vector<pair_multi_const> pairs_multi_consts;

  typedef std::pair<std::string, std::string> string_pair_type;
  typedef std::vector<string_pair_type> string_pairs_type;
  typedef std::pair<std::string, string_pairs_type> predicate_type;

  using Atom = std::string;

  struct Predicate;

  using Expression = boost::variant<Atom, Predicate>;

  struct Predicate
  {
    Atom function;
    std::vector<Expression> arguments;
  };

  struct Action
  {
    std::string name;
    string_pairs_type action_params;
    int duration;
    Expression precondition;
    Expression effect;
    Expression cond_breakup;
    Expression temp_breakup;
  };

  struct Domain
  {
    std::string name;
    std::vector<std::string> requirements;
    pairs_type types;
    pairs_multi_consts constants;
    std::vector<predicate_type> predicates;
    std::vector<Action> actions;
  };

}

BOOST_FUSION_ADAPT_STRUCT(
    pddl_parser::Domain,
    name,
    requirements,
    types,
    constants,
    predicates,
    actions
)

BOOST_FUSION_ADAPT_STRUCT(
    pddl_parser::Action,
    name,
    action_params,
    duration,
    precondition,
    effect,
    cond_breakup,
    temp_breakup
)

BOOST_FUSION_ADAPT_STRUCT(
    pddl_parser::Predicate,
    function,
    arguments
)

#endif
