/**
 * This file is part of pddl_parser
 *
 * @copyright Nils Adermann <naderman@naderman.de>
 *
 * For the full copyright and licensing information please review the LICENSE
 * file that was distributed with this source code.
 */

#ifndef PDDLQI_PARSER_PDDLGRAMMAR_H
#define PDDLQI_PARSER_PDDLGRAMMAR_H

#include "pddl_ast.h"
#include <boost/spirit/home/support/info.hpp>

#include <iostream>

#include <sstream>
#include <exception>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/repository/include/qi_distinct.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_fusion.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/spirit/include/phoenix_object.hpp>
#include <boost/spirit/include/phoenix_bind.hpp>

namespace pddl_parser
{
    class ParserException : std::exception
      {
        public:
            ParserException() : exception()
            {
                message = "Unknown ParserException";
            }

            template <typename Iterator>
            ParserException(Iterator start, Iterator current, Iterator end) :
                exception()
            {
                message = "Unknown error occured here: ";
                message += std::string(current, end);
            }

            template <typename Iterator>
            ParserException(const boost::spirit::info& expectedRule, Iterator start, Iterator end, Iterator current) :
                exception()
            {
                std::stringstream messageStream;
                messageStream << "Parse Error: Expected: ";
                messageStream << expectedRule;
                messageStream << " here: ";
                messageStream << std::string(current, end);

                message = messageStream.str();
            }

            ParserException(const ParserException& src)
            {
                message = src.message;
            }

            ParserException(const std::string& msg)
            {
                message = msg;
            }

            ParserException& operator=(const ParserException& rhs)
            {
                message = rhs.message;
                return *this;
            }

            virtual ~ParserException() throw() {}

            virtual const char* what() const throw()
            {
                return message.c_str();
            }
        protected:
            std::string message;
    };


    namespace Grammar
    {
        namespace fusion = boost::fusion;
        namespace phoenix = boost::phoenix;
        namespace qi = boost::spirit::qi;
        namespace ascii = boost::spirit::ascii;

        using qi::lexeme;
        using qi::lit;
        using qi::lazy;
        using qi::on_error;
        using qi::fail;
        using boost::spirit::repository::distinct;
        using ascii::char_;
        using namespace qi::labels;

        using phoenix::construct;
        using phoenix::val;
        using phoenix::ref;
        using phoenix::at_c;
        using phoenix::if_else;
        using phoenix::bind;
        using phoenix::push_back;
        using phoenix::insert;
        using phoenix::clear;
        using phoenix::begin;
        using phoenix::end;

        /** @class pddl_skipper
        * A skipper for PDDL files.
        * This skipper skips spaces and comments starting with ';'
        */
        template <typename Iterator>
        struct pddl_skipper : public qi::grammar<Iterator>
        {
	        pddl_skipper() : pddl_skipper::base_type(skip, "PDDL")
	        {
		        skip = ascii::space | (';' >> *(qi::char_ - qi::eol));
	        }
	        /** The actual skipping rule. */
	        qi::rule<Iterator> skip;
        };


        struct RequirementFlagSymbols_ :
            qi::symbols<char, pddl_parser::RequirementFlag::EnumType>
        {
            RequirementFlagSymbols_()
            {
                add
                    (":strips", RequirementFlag::eStrips)
                    (":negative-preconditions", RequirementFlag::eNegativePreconditions)
                    (":typing", RequirementFlag::typing)
                    (":action-costs", RequirementFlag::action_cost)
                    (":adl", RequirementFlag::adl)
                   ;
            }
        };

        struct OperatorSymbols_ :
            qi::symbols<char, pddl_parser::OperatorFlag::EnumType>
        {
            OperatorSymbols_()
            {
                this->add
                    ("and", OperatorFlag::conjunction)
                    ("not", OperatorFlag::negation)
                    ("or", OperatorFlag::disjunction)
                    ("when", OperatorFlag::condition)
                   ;
            }

        };

        void insert_typed_name_entities(TypedList& entities, const std::vector<std::string>& names, const std::string& type);
              
        template <typename Iterator, typename Skipper = pddl_skipper<Iterator>>
        struct BaseGrammar
        {
         
                
            BaseGrammar()
            {
               
                name %= lexeme[char_("a-zA-Z") >> *(char_("a-zA-Z0-9_-"))];
                name.name("name");

                variable %= lit('?') > name;
                variable.name("variable");

                type %= name;
                type.name("type");

                typedListExplicitType = (+(lazy(_r1)[push_back(_a, _1)]))
                     > lit('-')
                     > type[bind(&insert_typed_name_entities, _val, _a, _1)];
                typedListExplicitType.name("typedListExplicitType");
                char obj[] = "object";
                typedList =
                    (*(typedListExplicitType(_r1)[insert(_val, end(_val), begin(_1), end(_1))]))
                    > (*(lazy(_r1)[push_back(_val, construct<struct Entity>(_1, &obj[0]))]))
                    ;
                typedList.name("typedList");

                term = name[at_c<0>(_val) = false, at_c<1>(_val) = _1] |
                    variable[at_c<0>(_val) = true, at_c<1>(_val) = _1];
                term.name("term");

                atomicFormula = name[at_c<0>(_val) = _1] > (*term)[at_c<1>(_val) = _1];
                atomicFormula.name("atomicFormula");

                literal = atomicFormula[at_c<0>(_val) = false, at_c<1>(_val) = _1] |
                    ( lit("not") > lit('(') > atomicFormula[at_c<0>(_val) = true, at_c<1>(_val) = _1] > lit(')'));
                literal.name("literal");

                conditionalEffect = lit('(') >> goalDescription > lit(')')
                                    > effect;
                conditionalEffect.name("conditionalEffect");

                op %= operatorSymbols >> !(char_("a-zA-Z0-9_"));

                functionalEffect = op > ( +(effect) | conditionalEffect );
                functionalEffect.name("functionalEffect");

                actionCost = distinct(char_("a-zA-Z_0-9"))["increase"]
                            > lit('(')
                            >> name
                            > lit(')')
                            >> qi::int_;
                            

                effect = lit('(') >>
                            (functionalEffect | actionCost | atomicFormula )
                            > lit(')');
                effect.name("effect");

                functionalCondition = op > +(goalDescription);
                functionalCondition.name("functionalCondition");

                goalDescription = lit('(') >> (functionalCondition
                    | atomicFormula
                    ) > lit(')');
                goalDescription.name("goalDescription");

            }

            typedef qi::rule<Iterator, std::string(), Skipper> StringRule;

            qi::rule<Iterator, OperatorFlag::VectorType(), Skipper> op;
            qi::rule<Iterator, ConditionalEffect(), Skipper> conditionalEffect;
            qi::rule<Iterator, FunctionalCondition(), Skipper> functionalCondition;
            qi::rule<Iterator, FunctionalEffect(), Skipper> functionalEffect;
            qi::rule<Iterator, ActionCost(), Skipper> actionCost;
            qi::rule<Iterator, Effect(), Skipper> effect;
            qi::rule<Iterator, GoalDescription(), Skipper> goalDescription;
            qi::rule<Iterator, Literal(), Skipper> literal;
            qi::rule<Iterator, AtomicFormula(), Skipper> atomicFormula;
            qi::rule<Iterator, Term(), Skipper> term;
            qi::rule<Iterator, TypedList(StringRule), Skipper> typedList;
            qi::rule<Iterator, TypedList(StringRule), qi::locals<std::vector<std::string> >, Skipper> typedListExplicitType;
            StringRule type;
            StringRule name;
            StringRule variable;

            //qi::symbols<char,int> operatorSymbols;

            struct OperatorSymbols_ operatorSymbols;
        };

        template <typename Iterator, typename Skipper = pddl_skipper<Iterator>>
        struct Domain :
            qi::grammar<Iterator, PddlDomain(), Skipper>//, BaseGrammar<Iterator, Skipper>
        {
            //typedef BaseGrammar<Iterator,Skipper> base;

            Domain() :
                Domain::base_type(pddlDomain, "PDDL Domain")//, BaseGrammar<Iterator,Skipper>()
            {

                name %= lexeme[char_("a-zA-Z") >> *(char_("a-zA-Z0-9_-"))];
                name.name("name");

                variable %= lit('?') > name;
                variable.name("variable");

                type %= name;
                type.name("type");

                typedListExplicitType = (+(lazy(_r1)[push_back(_a, _1)]))
                     > lit('-')
                     > type[bind(&insert_typed_name_entities, _val, _a, _1)];
                typedListExplicitType.name("typedListExplicitType");
                char obj[] = "object";
                typedList =
                    (*(typedListExplicitType(_r1)[insert(_val, end(_val), begin(_1), end(_1))]))
                    > (*(lazy(_r1)[push_back(_val, construct<struct Entity>(_1, &obj[0]))]))
                    ;
                typedList.name("typedList");

                term = name[at_c<0>(_val) = false, at_c<1>(_val) = _1] |
                    variable[at_c<0>(_val) = true, at_c<1>(_val) = _1];
                term.name("term");

                atomicFormula = name[at_c<0>(_val) = _1] > (*term)[at_c<1>(_val) = _1];
                atomicFormula.name("atomicFormula");

                literal = atomicFormula[at_c<0>(_val) = false, at_c<1>(_val) = _1] |
                    ( lit("not") > lit('(') > atomicFormula[at_c<0>(_val) = true, at_c<1>(_val) = _1] > lit(')'));
                literal.name("literal");
                

                goalDescription %= lit('(') >> (functionalCondition[_val = _1]
                    | atomicFormula[_val = _1]
                    ) >> qi::eps >> lit(')');
                goalDescription.name("goalDescription");

                conditionalEffect = goalDescription >> effect;
                conditionalEffect.name("conditionalEffect");

                bool condition_flag;
                functionalEffect = operatorSymbols[at_c<0>(_val) = _1, phoenix::ref(condition_flag) = qi::_1 == OperatorFlag::condition] >> !(char_("a-zA-Z0-9_")) 
                                    >> ((qi::eps(phoenix::ref(condition_flag) == true) >> conditionalEffect)
                                        | (qi::eps(phoenix::ref(condition_flag) == false) >> +(effect)))[at_c<1>(_val) = _1];
                functionalEffect.name("functionalEffect");

                actionCost = distinct(char_("a-zA-Z_0-9"))["increase"]
                            > lit('(')
                            >> name
                            > lit(')')
                            >> qi::int_;
                            
                effect = lit('(') >>
                            (functionalEffect | actionCost | atomicFormula )
                            > lit(')');
                effect.name("effect");

                functionalCondition = operatorSymbols >> !(char_("a-zA-Z0-9_")) >> +(goalDescription);
                functionalCondition.name("functionalCondition");

                requireDef %= -(
                    lit('(')
                    >> lit(":requirements")
                    > (+(requirementFlagSymbols))
                    > lit(')')
                    );
                requireDef.name("requireDef");

                typesDef %= -(
                    lit('(')
                    >> lit(":types")
                    > typedList(phoenix::ref(name))
                    > lit(')')
                    );
                typesDef.name("typesDef");

                constantsDef %= -(
                    lit('(')
                    >> lit(":constants")
                    > typedList(phoenix::ref(name))
                    > lit(')')
                    );
                constantsDef.name("constantsDef");

                predicatesDef = -(
                    lit('(')
                    >> lit(":predicates")
                    > (+(lit('(')
                        > name[_a = _1]
                        >> typedList(phoenix::ref(variable))[_b = _1]
                        > lit(')'))[push_back(_val, construct<std::pair<std::string, TypedList> >(_a, _b))]
                    )
                    > lit(')')
                    );
                predicatesDef.name("predicatesDef");

                pddlAction =
                    lit('(')
                    > lit(":action")
                    > name[at_c<0>(_val) = _1]
                    > lit(":parameters")
                    > lit("(")
                    > typedList(phoenix::ref(variable))[at_c<1>(_val) = _1]
                    > lit(")")
                    > lit(":precondition")
                    >> (goalDescription[at_c<2>(_val) = _1] | (lit('(') > lit(')')))
                    > (
                        lit(":effect")
                        >> (effect[at_c<3>(_val) = _1] | (lit('(') > lit(')')))
                    )
                    > lit(')');
                pddlAction.name("pddlAction");

                actionsDef = *pddlAction;
                actionsDef.name("actionsDef");

                pddlDomain =
                    lit('(')
                    > lit("define")
                    > lit('(')
                    > lit("domain")
                    > name[at_c<0>(_val) = _1]
                    > lit(')')
                    > requireDef[at_c<1>(_val) = _1]
                    > typesDef[at_c<2>(_val) = _1]
                    > constantsDef[at_c<3>(_val) = _1]
                    > predicatesDef[at_c<4>(_val) = _1]
                    > actionsDef[at_c<5>(_val) = _1]
                    > lit(')');
                pddlDomain.name("pddlDomain");

                on_error<fail>
                (
                    pddlDomain,
                    construct<ParserException>(qi::_4, qi::_1, qi::_2, qi::_3)
                );
            }

            qi::rule<Iterator, PddlDomain(), Skipper> pddlDomain;
            qi::rule<Iterator, RequirementFlag::VectorType(), Skipper> requireDef;
            qi::rule<Iterator, TypedList(), Skipper> typesDef;
            qi::rule<Iterator, TypedList(), Skipper> constantsDef;
            qi::rule<Iterator, PddlAction(), Skipper> pddlAction;
            qi::rule<Iterator, ActionList(), Skipper> actionsDef;
            qi::rule<Iterator, PredicateList(), qi::locals<std::string, TypedList>, Skipper> predicatesDef;

            struct RequirementFlagSymbols_ requirementFlagSymbols;

            typedef qi::rule<Iterator, std::string(), Skipper> StringRule;

            qi::rule<Iterator, ConditionalEffect(), Skipper> conditionalEffect;

            qi::rule<Iterator, FunctionalCondition(), Skipper> functionalCondition;
            qi::rule<Iterator, FunctionalEffect(), Skipper> functionalEffect;
            qi::rule<Iterator, ActionCost(), Skipper> actionCost;
            qi::rule<Iterator, Effect(), Skipper> effect;
            qi::rule<Iterator, GoalDescription(), Skipper> goalDescription;
            qi::rule<Iterator, Literal(), Skipper> literal;
            qi::rule<Iterator, AtomicFormula(), Skipper> atomicFormula;
            qi::rule<Iterator, Term(), Skipper> term;
            qi::rule<Iterator, TypedList(StringRule), Skipper> typedList;
            qi::rule<Iterator, TypedList(StringRule), qi::locals<std::vector<std::string> >, Skipper> typedListExplicitType;
            StringRule type;
            StringRule name;
            StringRule variable;

            //qi::symbols<char,int> operatorSymbols;

            struct OperatorSymbols_ operatorSymbols;
        };

        template <typename Iterator, typename Skipper = pddl_skipper<Iterator>>
        struct Problem :
            qi::grammar<Iterator, PddlProblem(), Skipper>//, BaseGrammar<Iterator, Skipper>
        {
            //typedef BaseGrammar<Iterator,Skipper> base;

            Problem() :
                Problem::base_type(pddlProblem, "PDDL Domain")//, BaseGrammar<Iterator,Skipper>()
            {

                name %= lexeme[char_("a-zA-Z") >> *(char_("a-zA-Z0-9_-"))];
                name.name("name");

                variable %= lit('?') > name;
                variable.name("variable");

                type %= name;
                type.name("type");

                typedListExplicitType = (+(lazy(_r1)[push_back(_a, _1)]))
                     > lit('-')
                     > type[bind(&insert_typed_name_entities, _val, _a, _1)];
                typedListExplicitType.name("typedListExplicitType");
                char obj[] = "object";

                typedList =
                    (*(typedListExplicitType(_r1)[insert(_val, end(_val), begin(_1), end(_1))]))
                    > (*(lazy(_r1)[push_back(_val, construct<struct Entity>(_1, &obj[0]))]))
                    ;
                typedList.name("typedList");

                term = name[at_c<0>(_val) = false, at_c<1>(_val) = _1] |
                    variable[at_c<0>(_val) = true, at_c<1>(_val) = _1];
                term.name("term");

                atomicFormula = name[at_c<0>(_val) = _1] > (*term)[at_c<1>(_val) = _1];
                atomicFormula.name("atomicFormula");

                literal = atomicFormula[at_c<0>(_val) = false, at_c<1>(_val) = _1] |
                    ( lit("not") > lit('(') > atomicFormula[at_c<0>(_val) = true, at_c<1>(_val) = _1] > lit(')'));
                literal.name("literal");

                goalDescription %= lit('(') >> (functionalCondition[_val = _1]
                    | atomicFormula[_val = _1]
                    ) >> qi::eps >> lit(')');
                goalDescription.name("goalDescription");

                functionalCondition = operatorSymbols >> !(char_("a-zA-Z0-9_")) >> +(goalDescription);
                functionalCondition.name("functionalCondition");

                goalDef = lit('(') >> lit(":goal") >> goalDescription > lit(')');
                goalDef.name("goalDef");

                objectsDef %= -(
                    lit('(')
                    >> lit(":objects")
                    > typedList(phoenix::ref(name))
                    > lit(')')
                    );
                objectsDef.name("objectsDef");

                initialFacts = (
                    lit('(')
                    >> lit(":init")
                    > (+(lit('(')
                        >> atomicFormula 
                        > lit(')'))
                    )
                    > lit(')')
                    );
                initialFacts.name("initialFacts");

                pddlProblem =
                    lit('(')
                    > lit("define")
                    > lit('(')
                    > lit("problem")
                    > name[at_c<0>(_val) = _1]
                    > lit(')')
                    > lit('(')
                    > lit("(:domain")
                    > name[at_c<1>(_val) = _1]
                    > objectsDef[at_c<2>(_val) = _1]
                    > initialFacts[at_c<3>(_val) = _1]
                    > goalDef[at_c<4>(_val) = _1]
                    > lit(')');
                pddlProblem.name("pddlProblem");

                on_error<fail>
                (
                    pddlProblem,
                    construct<ParserException>(qi::_4, qi::_1, qi::_2, qi::_3)
                );
            }

            qi::rule<Iterator, PddlProblem(), Skipper> pddlProblem;
            qi::rule<Iterator, TypedList(), Skipper> objectsDef;
            qi::rule<Iterator, PredicateList(), qi::locals<std::string, TypedList>, Skipper> predicatesDef;
            qi::rule<Iterator, FactList(), Skipper> initialFacts;
            typedef qi::rule<Iterator, std::string(), Skipper> StringRule;
            qi::rule<Iterator, FunctionalCondition(), Skipper> functionalCondition;
            qi::rule<Iterator, GoalDescription(), Skipper> goalDescription;
            qi::rule<Iterator, GoalDescription(), Skipper> goalDef;
            qi::rule<Iterator, Literal(), Skipper> literal;
            qi::rule<Iterator, AtomicFormula(), Skipper> atomicFormula;
            qi::rule<Iterator, Term(), Skipper> term;
            qi::rule<Iterator, TypedList(StringRule), Skipper> typedList;
            qi::rule<Iterator, TypedList(StringRule), qi::locals<std::vector<std::string> >, Skipper> typedListExplicitType;
            StringRule type;
            StringRule name;
            StringRule variable;

            //qi::symbols<char,int> operatorSymbols;

            struct OperatorSymbols_ operatorSymbols;
        };
    }
}

#endif
