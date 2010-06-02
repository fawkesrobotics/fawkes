 
/***************************************************************************
 *  tolua_generator.h - ToLua++ Interface generator
 *
 *  Created: Tue Mar 11 15:30:09 2008
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

#ifndef __INTERFACES_GENERATOR_TOLUA_GENERATOR_H_
#define __INTERFACES_GENERATOR_TOLUA_GENERATOR_H_

#include "field.h"
#include "constant.h"
#include "enum_constant.h"
#include "message.h"
#include "pseudomap.h"

#include <vector>
#include <string>
#include <stdio.h>

class ToLuaInterfaceGenerator
{
 public:
  ToLuaInterfaceGenerator(std::string directory, std::string interface_name,
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
  ~ToLuaInterfaceGenerator();

  void write_toluaf(FILE *f);

  void write_header(FILE *f, std::string filename);
  void write_constants_h(FILE *f);
  void write_messages_h(FILE *f);
  void write_superclass_h(FILE *f);
  void write_methods_h(FILE *f,
		       std::string /* indent space */ is,
		       std::vector<InterfaceField> fields);
  void write_methods_h(FILE *f, std::string /* indent space */ is,
		       std::vector<InterfaceField> fields,
		       std::vector<InterfacePseudoMap> pseudo_maps);
  void write_message_ctor_dtor_h(FILE *f, std::string /* indent space */ is,
				 std::string classname,
				 std::vector<InterfaceField> fields);
  void write_ctor_dtor_h(FILE *f, std::string /* indent space */ is,
			 std::string classname);

  void generate();

  const char * convert_type(std::string c_type);

 private:
  std::vector<InterfaceConstant>     constants;
  std::vector<InterfaceEnumConstant> enum_constants;
  std::vector<InterfaceField>        data_fields;
  std::vector<InterfacePseudoMap>    pseudo_maps;
  std::vector<InterfaceMessage>      messages;

  std::string dir;
  std::string filename_tolua;
  std::string filename_h;
  std::string class_name;
  std::string gendate;
  std::string author;
  std::string year;
  std::string creation_date;
  std::string data_comment;

  const unsigned char *hash;
  size_t hash_size;
};


#endif
