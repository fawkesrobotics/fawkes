/**
 * This file is part of pddl_parser
 *
 * @copyright Nils Adermann <naderman@naderman.de>
 *
 * For the full copyright and licensing information please review the LICENSE
 * file that was distributed with this source code.
 */

#ifndef PDDLQI_PARSER_PARSER_H
#define BOOST_SPIRIT_DEBUG
#define PDDLQI_PARSER_PARSER_H

#include "pddl_grammar.h"
#include <iostream>

namespace pddl_parser
{
    class Parser
    {
        public:
            PddlDomain parseDomain(const std::string& input)
            {
                return parse<Grammar::Domain<std::string::const_iterator>, Grammar::pddl_skipper<std::string::const_iterator>, PddlDomain>(input);
            }

            PddlProblem parseProblem(const std::string& input)
            {
                return parse<Grammar::Problem<std::string::const_iterator>, Grammar::pddl_skipper<std::string::const_iterator>, PddlProblem>(input);
            }

            template <typename Grammar, typename Skipper, typename Attribute>
            Attribute parse(const std::string& input)
            {
                Skipper skipper;

                Grammar grammar;

                Attribute data;

                std::string::const_iterator iter = input.begin();
                std::string::const_iterator end = input.end();

                bool r = phrase_parse(
                    iter,
                    end,
                    grammar,
                    skipper,
                    data
                );

                if (!r || iter != end)
                {
                    //std::cout << "Failed randomly?" << std::endl;
                    throw ParserException(input.begin(), iter, end);
                }

                return data;
            }
    };
}
#endif
