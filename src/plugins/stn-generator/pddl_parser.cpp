    
/***************************************************************************     
 *  pddl_parser.cpp
 *     
 *  Created: Fri 19 May 2017 11:10:01 CEST
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

#include <fstream>
#include <streambuf>

#include "pddl_parser.h"

namespace pddl_parser {

Domain
PddlParser::parseDomain(const std::string pddl_domain)
{
  typedef std::string::const_iterator iterator_type;
  typedef pddl_parser::grammar::domain_parser<iterator_type> grammar;
  typedef pddl_parser::grammar::pddl_skipper<iterator_type> skipper;

  grammar g;
  skipper s;
  Domain dom;

  std::string::const_iterator iter = pddl_domain.begin();
  std::string::const_iterator end = pddl_domain.end();
  bool r = phrase_parse(iter, end, g, s, dom);

  if ( !r ) {
    //TODO implement correct exception and behavior
    throw ("Parsing PDDL string failed!");
  }


  return dom;
}

}

int main()
{
  std::ifstream t("domain.pddl");
  std::string str;

  t.seekg(0, std::ios::end);
  str.reserve(t.tellg());
  t.seekg(0, std::ios::beg);

  str.assign((std::istreambuf_iterator<char>(t)),
      std::istreambuf_iterator<char>());

  pddl_parser::Domain dom = pddl_parser::PddlParser::parseDomain(str);

  std::cout << "success" << std::endl;

    std::cout << dom.name << std::endl;
    std::cout << "requirements:" << std::endl;
    for ( std::string s : dom.requirements ) {
      std::cout << "\t" << s << std::endl;
    }
    std::cout << "types:" << std::endl;
    for ( std::pair<std::string, std::string> p : dom.types ) {
      std::cout << "\t" << p.first << " - " << p.second << std::endl;
    }
    std::cout << "constants:" << std::endl;
    for ( std::pair<std::vector<std::string>,std::string> p : dom.constants ) {
      std::cout << "\t";
      for ( std::string s : p.first ) {
       std::cout << s << " ";
      }
      std::cout  << "- " << p.second << std::endl;
    }
    std::cout << "predicates:" << std::endl;
    for ( std::pair<std::string, std::vector<std::pair<std::string, std::string> > > p1 : dom.predicates ) {
      std::cout << "\t" << p1.first << std::endl;
      for ( std::pair<std::string, std::string> p2 : p1.second ) {
        std::cout << "\t\t" << p2.first << ":" << p2.second << std::endl;
      }
    }
    std::cout << "actions:" << std::endl;
    for (  auto a : dom.actions ) {
      std::cout << "\t" << a.name << std::endl;
      for ( std::pair<std::string, std::string> p : a.action_params ) {
        std::cout << "\t\t" << p.first << ":" << p.second << std::endl;
      }
    }

}
