
/***************************************************************************
 *  grammar.h
 *
 *  Created: Fri 19 May 2017 14:07:29 CEST
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

#ifndef __PLUGINS_PDDL_GRAMMAR_H
#define __PLUGINS_PDDL_GRAMMAR_H

#include "pddl_ast.h"

namespace pddl_parser {
  namespace grammar {
    template<typename Iterator>
    struct pddl_skipper : public qi::grammar<Iterator> {

      pddl_skipper() : pddl_skipper::base_type(skip, "PDDL") {
          skip = ascii::space | (';' >> *(qi::char_) >> '\n');
      }
      qi::rule<Iterator> skip;
    };

    template <typename Iterator, typename Skipper = pddl_skipper<Iterator>>
    struct domain_parser : qi::grammar<Iterator, Domain(), Skipper>
    {
      domain_parser() : domain_parser::base_type(domain)
      {
        using namespace qi;
        using ascii::char_;
        using ascii::alnum;
        using ascii::blank;

        name_type = lexeme[ alnum >> *(alnum|char_('-')|char_('_')) ];

        domain_name = '(' >> lit("define")
          >> '(' >> lit("domain") >> +(char_ - ')') >> ')';

        requirements = '(' >> lit(":requirements")
          >> *(':' >> lexeme[*qi::alnum]) >> ')';

        type_pair = name_type >> -('-' >> name_type);
        types = '(' >> lit(":types") >> +type_pair >> ')';

        constant_value_list = +name_type;
        constant_multi_pair = constant_value_list >> -('-' >> name_type);
        constants = '(' >> lit(":constants") >> +constant_multi_pair >> ')';

        param_pair = '?' >> name_type >> '-' >> name_type;
        param_pairs = +param_pair;
        pred = '(' >> name_type >> -param_pairs >> ')';
        predicates = '(' >> lit(":predicates") >> +pred >> ')';

        atom = +(graph - '(' - ')');
        predicate = '(' >> atom >> *expression >> ')';
        expression = atom | predicate;
        temp_breakup = lit(":temporal-breakup") >> expression;
        cond_breakup = lit(":conditional-breakup") >> expression;
        effects = lit(":effect") >> expression;
        preconditions = lit(":precondition") >> expression;
        action_params = lit(":parameters") >> '(' >> +param_pair >> ')';
        action = '(' >> lit(":durative-action")
          >> name_type
          >> action_params
          >> preconditions
          >> -cond_breakup
          >> -temp_breakup
          >> effects
          >> ')';
        actions = +action;

        domain =
          domain_name
          >> requirements
          >> -types
          >> -constants
          >> predicates
          >> actions
          // There should be a closing paranthes here, but parsing will
          // fail if it is included
          ;
      }

      qi::rule<Iterator, std::string(), Skipper> name_type;

      qi::rule<Iterator, std::string(), Skipper> domain_name;

      qi::rule<Iterator, std::vector<std::string>(), Skipper> requirements;

      qi::rule<Iterator, pairs_type(), Skipper> types;
      qi::rule<Iterator, pair_type(), Skipper> type_pair;

      qi::rule<Iterator, type_list(), Skipper> constant_value_list, predicate_params;
      qi::rule<Iterator, pair_multi_const(), Skipper> constant_multi_pair;
      qi::rule<Iterator, pairs_multi_consts(), Skipper> constants;

      qi::rule<Iterator, string_pair_type(), Skipper> param_pair;
      qi::rule<Iterator, string_pairs_type(), Skipper> param_pairs;
      qi::rule<Iterator, predicate_type(), Skipper> pred;
      qi::rule<Iterator, std::vector<predicate_type>(), Skipper> predicates;


      qi::rule<Iterator, Atom()> atom;
      qi::rule<Iterator, Predicate(), Skipper> predicate;
      qi::rule<Iterator, Expression(), Skipper> expression;
      qi::rule<Iterator, Expression(), Skipper> preconditions, effects,
        temp_breakup, cond_breakup;
      qi::rule<Iterator, string_pairs_type(), Skipper> action_params;
      qi::rule<Iterator, Action(), Skipper> action;
      qi::rule<Iterator, std::vector<Action>(), Skipper> actions;

      qi::rule<Iterator, Domain(), Skipper> domain;
    };
  }
}

#endif
