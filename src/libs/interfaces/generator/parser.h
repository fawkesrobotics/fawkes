 
/***************************************************************************
 *  parser.h - Interface config parser
 *
 *  Created: Tue Oct 10 17:29:33 2006
 *  Copyright  2006-2015  Tim Niemueller [www.niemueller.de]
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

#ifndef __INTERFACES_GENERATOR_PARSER_H_
#define __INTERFACES_GENERATOR_PARSER_H_

#include "field.h"
#include "constant.h"
#include "enum_constant.h"
#include "message.h"
#include "pseudomap.h"

#include <vector>
#include <string>

namespace xmlpp {
  class DomParser;
  class Node;
}

class InterfaceParser
{
 public:
  InterfaceParser(std::string config_filename);
  ~InterfaceParser();

  std::vector<InterfaceField>  getFields(xmlpp::Node *node, const std::set<std::string> &reserved_names);
  std::vector<InterfacePseudoMap> getPseudoMaps(xmlpp::Node *node,
						std::vector<InterfaceField> &fields);
  void parse();

  void printFields(std::vector<InterfaceField> &fields);
  void printPseudoMaps(std::vector<InterfacePseudoMap> &pseudo_maps);
  void print();
  void printParsed(std::vector<InterfaceConstant> &     constants,
		   std::vector<InterfaceEnumConstant> & enum_constants,
		   std::vector<InterfaceField> &        data_fields,
		   std::vector<InterfacePseudoMap> &    pseudo_maps,
		   std::vector<InterfaceMessage> &      messages);

  std::string                         getInterfaceName();
  std::string                         getInterfaceAuthor();
  std::string                         getInterfaceYear();
  std::string                         getInterfaceCreationDate();
  std::vector<InterfaceConstant>      getConstants();
  std::vector<InterfaceEnumConstant>  getEnumConstants();
  std::vector<InterfaceField>         getDataFields();
  std::vector<InterfacePseudoMap>     getPseudoMaps();
  std::string                         getDataComment();
  std::vector<InterfaceMessage>       getMessages();

 private:
  xmlpp::DomParser *dom;
  xmlpp::Node      *root;
  std::string       name;
  std::string       author;
  std::string       year;
  std::string       creation_date;
  std::string       data_comment;

  std::vector<InterfaceConstant>     constants;
  std::vector<InterfaceEnumConstant> enum_constants;
  std::vector<InterfaceField>        data_fields;
  std::vector<InterfacePseudoMap>    pseudo_maps;
  std::vector<InterfaceMessage>      messages;

};


#endif
