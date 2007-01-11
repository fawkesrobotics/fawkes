 
/***************************************************************************
 *  parser.h - Interface config parser
 *
 *  Generated: Tue Oct 10 17:29:33 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __INTERFACES_GENERATOR_PARSER_H_
#define __INTERFACES_GENERATOR_PARSER_H_

#include <interfaces/generator/field.h>
#include <interfaces/generator/constant.h>
#include <interfaces/generator/enum_constant.h>
#include <interfaces/generator/message.h>

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

  std::vector<InterfaceField>  getFields(xmlpp::Node *node);
  void parse();

  void printFields(std::vector<InterfaceField> &fields);
  void print();
  void printParsed(std::vector<InterfaceConstant> &     constants,
		   std::vector<InterfaceEnumConstant> & enum_constants,
		   std::vector<InterfaceField> &        data_fields,
		   std::vector<InterfaceMessage> &      messages);

  std::string                         getInterfaceName();
  std::string                         getInterfaceAuthor();
  std::string                         getInterfaceYear();
  std::string                         getInterfaceCreationDate();
  std::vector<InterfaceConstant>      getConstants();
  std::vector<InterfaceEnumConstant>  getEnumConstants();
  std::vector<InterfaceField>         getDataFields();
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
  std::vector<InterfaceMessage>      messages;

};


#endif
