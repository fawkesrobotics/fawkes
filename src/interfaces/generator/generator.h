 
/***************************************************************************
 *  generator.h - Interface generator
 *
 *  Generated: Thu Oct 12 01:59:02 2006
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

#ifndef __INTERFACES_GENERATOR_GENERATOR_H_
#define __INTERFACES_GENERATOR_GENERATOR_H_

#include <interfaces/generator/field.h>
#include <interfaces/generator/constant.h>
#include <interfaces/generator/enum_constant.h>
#include <interfaces/generator/message.h>

#include <vector>
#include <string>
#include <stdio.h>

class InterfaceGenerator
{
 public:
  InterfaceGenerator(std::string directory, std::string interface_name,
		     std::string config_basename, std::string author,
		     std::string year, std::string creation_date,
		     std::string data_comment);
  ~InterfaceGenerator();

  void setConstants(const std::vector<InterfaceConstant> &constants);
  void setEnumConstants(const std::vector<InterfaceEnumConstant> &enum_constants);
  void setDataFields(const std::vector<InterfaceField> &data_fields);
  void setMessages(const std::vector<InterfaceMessage> &messages);


  void write_h(FILE *f);
  void write_cpp(FILE *f);

  void write_constants_cpp(FILE *f);
  void write_constants_h(FILE *f);

  void write_messages_cpp(FILE *f);
  void write_messages_h(FILE *f);
  void write_ctor_dtor_h(FILE *f,  std::string /* indent space */ is,
			 std::string classname);
  void write_ctor_dtor_cpp(FILE *f, std::string classname, std::string super_class,
			   std::string inclusion_prefix);

  void write_message_ctor_dtor_h(FILE *f,  std::string /* indent space */ is,
				 std::string classname,
				 std::vector<InterfaceField> fields);
  void write_message_ctor_dtor_cpp(FILE *f, std::string classname, std::string super_class,
				   std::string inclusion_prefix,
				   std::vector<InterfaceField> fields);

  void write_methods_h(FILE *f,
		       std::string /* indent space */ is,
		       std::vector<InterfaceField> fields);
  void write_methods_cpp(FILE *f,
			 std::string interface_classname,
			 std::string classname,
			 std::vector<InterfaceField> fields,
			 std::string inclusion_prefix);

  void write_management_funcs_cpp(FILE *f);


  void write_struct(FILE *f, std::string name, std::string /* indent space */ is,
		    std::vector<InterfaceField> fields);

  void write_header(FILE *f, std::string filename);
  void write_deflector(FILE *f);
  void generate();

 private:
  std::vector<InterfaceConstant>     constants;
  std::vector<InterfaceEnumConstant> enum_constants;
  std::vector<InterfaceField>        data_fields;
  std::vector<InterfaceMessage>      messages;

  std::string dir;
  std::string filename_cpp;
  std::string filename_h;
  std::string filename_o;
  std::string class_name;
  std::string deflector;
  std::string gendate;
  std::string author;
  std::string year;
  std::string creation_date;
  std::string data_comment;
};


#endif
