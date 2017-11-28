/***************************************************************************
 *  main.cpp - PDDL Parser
 *
 *  Created: Fri 13 Oct 2017 14:50:44 CEST 14:50
 *  Copyright  2017  Matthias Loebach
 *                   Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include <pddl_parser/pddl_parser.h>

#include <iostream>
#include <fstream>

int main()
{
  std::ifstream t("domain.pddl");
  std::string str;

  t.seekg(0, std::ios::end);
  str.reserve(t.tellg());
  t.seekg(0, std::ios::beg);

  str.assign((std::istreambuf_iterator<char>(t)),
      std::istreambuf_iterator<char>());

  pddl_parser::Domain dom;
  try {
  dom = pddl_parser::PddlParser::parseDomain(str);
  } catch (pddl_parser::PddlParserException &e) {
    std::cout << "Error occurred during parsing: "
      << e.what_no_backtrace() << std::endl;
    return 1;
  }

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
