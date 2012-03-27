 
/***************************************************************************
 *  cpp_generator.h - C++ Interface generator
 *
 *  Created: Thu Oct 12 01:59:02 2006
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
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

#ifndef __INTERFACES_GENERATOR_CPP_GENERATOR_H_
#define __INTERFACES_GENERATOR_CPP_GENERATOR_H_

#include "field.h"
#include "constant.h"
#include "enum_constant.h"
#include "message.h"
#include "pseudomap.h"

#include <vector>
#include <string>
#include <stdio.h>

class CppInterfaceGenerator
{
 public:
  CppInterfaceGenerator(std::string directory, std::string interface_name,
			std::string config_basename, std::string author,
			std::string year, std::string creation_date,
			std::string data_comment,
			const unsigned char *hash, size_t hash_size,
			const std::vector<InterfaceConstant> &constants,
			const std::vector<InterfaceEnumConstant> &enum_constants,
			const std::vector<InterfaceField> &data_fields,
			const std::vector<InterfacePseudoMap> &pseudo_maps,
			const std::vector<InterfaceMessage> &messages
			);
  ~CppInterfaceGenerator();

  void write_h(FILE *f);
  void write_cpp(FILE *f);

  void write_constants_cpp(FILE *f);
  void write_constants_h(FILE *f);
  void write_enum_constants_tostring_cpp(FILE *f);

  void write_messages_cpp(FILE *f);
  void write_messages_h(FILE *f);
  void write_ctor_dtor_h(FILE *f,  std::string /* indent space */ is,
			 std::string classname);
  void write_ctor_dtor_cpp(FILE *f, std::string classname, std::string super_class,
			   std::string inclusion_prefix, std::vector<InterfaceField> fields,
			   std::vector<InterfaceMessage> messages);

  void write_message_ctor_dtor_h(FILE *f,  std::string /* indent space */ is,
				 std::string classname,
				 std::vector<InterfaceField> fields);
  void write_message_ctor_dtor_cpp(FILE *f, std::string classname, std::string super_class,
				   std::string inclusion_prefix,
				   std::vector<InterfaceField> fields);
  void write_message_clone_method_h(FILE *f, std::string is);
  void write_message_clone_method_cpp(FILE *f, std::string classname);


  void write_methods_h(FILE *f,
		       std::string /* indent space */ is,
		       std::vector<InterfaceField> fields);
  void write_methods_cpp(FILE *f,
			 std::string interface_classname,
			 std::string classname,
			 std::vector<InterfaceField> fields,
			 std::string inclusion_prefix,
			 bool write_data_changed);

  void write_create_message_method_cpp(FILE *f);
  void write_copy_value_method_cpp(FILE *f);
  void write_enum_tostring_method_cpp(FILE *f);
  void write_basemethods_h(FILE *f, std::string is);
  void write_basemethods_cpp(FILE *f);

  void write_methods_h(FILE *f,
		       std::string /* indent space */ is,
		       std::vector<InterfaceField> fields,
		       std::vector<InterfacePseudoMap> pseudo_maps);
  void write_methods_cpp(FILE *f,
			 std::string interface_classname,
			 std::string classname,
			 std::vector<InterfaceField> fields,
			 std::vector<InterfacePseudoMap> pseudo_maps,
			 std::string inclusion_prefix);

  void write_management_funcs_cpp(FILE *f);

  void write_add_fieldinfo_calls(FILE *f, std::vector<InterfaceField> &fields);


  void write_struct(FILE *f, std::string name, std::string /* indent space */ is,
		    std::vector<InterfaceField> fields);

  void write_header(FILE *f, std::string filename);
  void write_deflector(FILE *f);
  void generate();

 private:
  std::vector<InterfaceConstant>     constants;
  std::vector<InterfaceEnumConstant> enum_constants;
  std::vector<InterfaceField>        data_fields;
  std::vector<InterfacePseudoMap>    pseudo_maps;
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

  const unsigned char *hash;
  size_t hash_size;
};


#endif
