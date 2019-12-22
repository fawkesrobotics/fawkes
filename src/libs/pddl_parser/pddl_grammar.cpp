#include "pddl_grammar.h"

using namespace pddl_parser;
namespace phoenix = boost::phoenix;

void
pddl_parser::Grammar::insert_typed_name_entities(TypedList &                     entities,
                                                 const std::vector<std::string> &names,
                                                 const std::string &             type)
{
	std::for_each(names.begin(),
	              names.end(),
	              (phoenix::push_back(phoenix::ref(entities),
	                                  phoenix::construct<struct Entity>(phoenix::arg_names::_1,
	                                                                    phoenix::ref(type)))));
}