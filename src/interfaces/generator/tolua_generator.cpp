 
/***************************************************************************
 *  tolua_generator.cpp - ToLua++ Interface generator
 *
 *  Created: Tue Mar 11 15:33:26 2006
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

#include "tolua_generator.h"
#include "exceptions.h"

#include <utils/misc/string_conversions.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <time.h>
#include <fstream>

using namespace std;


/** @class ToLuaInterfaceGenerator <interfaces/generator/tolua_generator.h>
 * Generator that transforms input from the InterfaceParser into valid
 * ToLua++ package file.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param directory Directory where to create the files
 * @param interface_name name of the interface, should end with Interface
 * @param config_basename basename of the config without suffix
 * @param author author of interface
 * @param year year of copyright
 * @param creation_date user-supplied creation date of interface
 * @param data_comment comment in data block.
 * @param hash MD5 hash of the config file that was used to generate the interface
 * @param hash_size size in bytes of hash
 * @param constants constants
 * @param enum_constants constants defined as an enum
 * @param data_fields data fields of the interface
 * @param pseudo_maps pseudo maps of the interface
 * @param messages messages defined in the interface
 */
ToLuaInterfaceGenerator::ToLuaInterfaceGenerator(std::string directory, std::string interface_name,
						 std::string config_basename, std::string author,
						 std::string year, std::string creation_date,
						 std::string data_comment,
						 const unsigned char *hash, size_t hash_size,
						 const std::vector<InterfaceConstant> &constants,
						 const std::vector<InterfaceEnumConstant> &enum_constants,
						 const std::vector<InterfaceField> &data_fields,
						 const std::vector<InterfacePseudoMap> &pseudo_maps,
						 const std::vector<InterfaceMessage> &messages
						 )
{
  this->dir    = directory;
  if ( dir.find_last_of("/") != (dir.length() - 1) ) {
    dir += "/";
  }
  this->author = author;
  this->year   = year;
  this->creation_date = creation_date;
  this->data_comment  = data_comment;
  this->hash = hash;
  this->hash_size = hash_size;
  this->constants = constants;
  this->enum_constants = enum_constants;
  this->data_fields = data_fields;
  this->pseudo_maps = pseudo_maps;
  this->messages = messages;

  filename_tolua = config_basename + ".tolua";
  filename_h     = config_basename + ".h";

  if ( interface_name.find("Interface", 0) == string::npos ) {
    // append Interface
    class_name = interface_name + "Interface";
  } else {
    class_name = interface_name;
  }
}


/** Destructor */
ToLuaInterfaceGenerator::~ToLuaInterfaceGenerator()
{
}


/** Convert C type to Lua type.
 * tolua++ does not deal well with stdint types, therefore we convert them
 * to "traditional" types.
 * @param c_type C type to convert
 * @return constant string of the Lua compatible type
 */
const char *
ToLuaInterfaceGenerator::convert_type(std::string c_type)
{
  if (c_type == "uint8_t") {
    return "unsigned char";
  } else if (c_type == "uint16_t") {
    return "unsigned short";
  } else if (c_type == "uint32_t") {
    return "unsigned int";
  } else if (c_type == "uint64_t") {
#if __WORDSIZE == 64
    return "unsigned long";
#else
    return "unsigned long long";
#endif
  } else if (c_type == "int8_t") {
    return "char";
  } else if (c_type == "int16_t") {
    return "short";
  } else if (c_type == "int32_t") {
    return "int";
  } else if (c_type == "int64_t") {
#if __WORDSIZE == 64
    return "long";
#else
    return "long long";
#endif
  } else if (c_type == "uint8_t *") {
    return "unsigned char *";
  } else if (c_type == "uint16_t *") {
    return "unsigned short *";
  } else if (c_type == "uint32_t *") {
    return "unsigned int *";
  } else if (c_type == "uint64_t *") {
#if __WORDSIZE == 64
    return "unsigned long *";
#else
    return "unsigned long long *";
#endif
  } else if (c_type == "int8_t *") {
    return "char *";
  } else if (c_type == "int16_t *") {
    return "short *";
  } else if (c_type == "int32_t *") {
    return "int *";
  } else if (c_type == "int64_t *") {
#if __WORDSIZE == 64
    return "long *";
#else
    return "long long *";
#endif
  } else {
    return c_type.c_str();
  }
}



/** Write header to file.
 * @param f file to write to
 * @param filename name of file
 */
void
ToLuaInterfaceGenerator::write_header(FILE *f, std::string filename)
{
  fprintf(f, "\n/***************************************************************************\n");
  fprintf(f, " *  %s - Fawkes BlackBoard Interface - %s - tolua++ wrapper\n", filename.c_str(), class_name.c_str());
  fprintf(f, " *\n");
  if ( creation_date.length() > 0 ) {
    fprintf(f, " *  Interface created: %s\n", creation_date.c_str());
  }
  fprintf(f, " *  Templated created:   Thu Oct 12 10:49:19 2006\n");
  fprintf(f, " *  Copyright  %s  %s\n", year.c_str(),
	  ((author.length() > 0) ? author.c_str() : "AllemaniACs RoboCup Team") );
  fprintf(f, " *\n");
  fprintf(f, " ****************************************************************************/\n\n");
  fprintf(f, "/*\n");
  fprintf(f, " *  This program is free software; you can redistribute it and/or modify\n");
  fprintf(f, " *  it under the terms of the GNU General Public License as published by\n");
  fprintf(f, " *  the Free Software Foundation; either version 2 of the License, or\n");
  fprintf(f, " *  (at your option) any later version.\n");
  fprintf(f, " *\n");
  fprintf(f, " *  This program is distributed in the hope that it will be useful,\n");
  fprintf(f, " *  but WITHOUT ANY WARRANTY; without even the implied warranty of\n");
  fprintf(f, " *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n");
  fprintf(f, " *  GNU Library General Public License for more details.\n");
  fprintf(f, " *\n");
  fprintf(f, " *  You should have received a copy of the GNU General Public License\n");
  fprintf(f, " *  along with this program; if not, write to the Free Software Foundation,\n");
  fprintf(f, " *  Inc., 51 Franklin Street, Fifth floor, Boston, MA 02111-1307, USA.\n");
  fprintf(f, " */\n\n");
}


/** Write constants to h file
 * @param f file to write to
 */
void
ToLuaInterfaceGenerator::write_constants_h(FILE *f)
{
  for ( vector<InterfaceConstant>::iterator i = constants.begin(); i != constants.end(); ++i) {
    fprintf(f, "  static const %s %s;\n", convert_type(i->getType()),
	    i->getName().c_str());
  }
  fprintf(f, "\n");

  for ( vector<InterfaceEnumConstant>::iterator i = enum_constants.begin(); i != enum_constants.end(); ++i) {
    fprintf(f, "  typedef enum {\n");
    vector< pair<string,string> > items = (*i).getItems();
    vector< pair<string,string> >::iterator j = items.begin();
    while (j != items.end()) {
      fprintf(f, "    %s", (*j).first.c_str());
      ++j;
      if ( j != items.end() ) {
	fprintf(f, ",\n");
      } else {
	fprintf(f, "\n");
      }
    }
    fprintf(f, "  } %s;\n\n", (*i).getName().c_str());
  }
}


/** Write messages to h file.
 * @param f file to write to
 */
void
ToLuaInterfaceGenerator::write_messages_h(FILE *f)
{
  for (vector<InterfaceMessage>::iterator i = messages.begin(); i != messages.end(); ++i) {
    fprintf(f, "  class %s : public Message\n"
	    "  {\n", (*i).getName().c_str());
    write_message_ctor_dtor_h(f, "    ", (*i).getName(), (*i).getFields());
    write_methods_h(f, "    ", (*i).getFields());

    fprintf(f, "  };\n\n");
  }

}


/** Write constructor and destructor to h file.
 * @param f file to write to
 * @param is indentation space
 * @param classname name of class
 */
void
ToLuaInterfaceGenerator::write_ctor_dtor_h(FILE *f, std::string /* indent space */ is,
					 std::string classname)
{
  fprintf(f,
	  "%s%s();\n"
	  "%s~%s();\n\n",
	  is.c_str(), classname.c_str(),
	  is.c_str(), classname.c_str());
}


/** Write constructor and destructor for message to h file.
 * @param f file to write to
 * @param is indentation space
 * @param classname name of class
 * @param fields vector of data fields of message
 */
void
ToLuaInterfaceGenerator::write_message_ctor_dtor_h(FILE *f, std::string /* indent space */ is,
						 std::string classname,
						 std::vector<InterfaceField> fields)
{
  vector<InterfaceField>::iterator i;

  if ( fields.size() > 0 ) {

    fprintf(f, "%s%s(", is.c_str(), classname.c_str());

    i = fields.begin();
    while (i != fields.end()) {
      fprintf(f, "%s ini_%s",
	      convert_type(i->getAccessType()), i->getName().c_str());
      ++i;
      if ( i != fields.end() ) {
	fprintf(f, ", ");
      }
    }

    fprintf(f, ");\n");
  }


  write_ctor_dtor_h(f, is, classname);
}

/** Write superclass methods.
 * @param f file to write to
 */
void
ToLuaInterfaceGenerator::write_superclass_h(FILE *f)
{
  fprintf(f,
          "  bool                    oftype(const char *interface_type) const;\n"
          "  const void *            datachunk() const;\n"
          "  unsigned int            datasize() const;\n"
          "  const char *            type() const;\n"
          "  const char *            id() const;\n"
          "  const char *            uid() const;\n"
          "  unsigned int            serial() const;\n"
          "  unsigned int            mem_serial() const;\n"
          "  bool                    operator== (Interface &comp) const;\n"
          "  const unsigned char *   hash() const;\n"
          "  int                     hash_size() const;\n"
          "  const char *            hash_printable() const;\n"
          "  bool                    is_writer() const;\n"

          "  void                    set_from_chunk(void *chunk);\n"

          "  virtual Message *   create_message(const char *type) const = 0;\n"

          "  void          read();\n"
          "  void          write();\n"

          "  bool          has_writer() const;\n"
          "  unsigned int  num_readers() const;\n"


          "  unsigned int  msgq_enqueue_copy(Message *message);\n"
          "  void          msgq_remove(Message *message);\n"
          "  void          msgq_remove(unsigned int message_id);\n"
          "  unsigned int  msgq_size();\n"
          "  void          msgq_flush();\n"
          "  void          msgq_lock();\n"
          "  bool          msgq_try_lock();\n"
          "  void          msgq_unlock();\n"
          "  void          msgq_pop();\n"
          "  Message *     msgq_first();\n"
          "  bool          msgq_empty();\n"
          "\n");
}

/** Write methods to h file.
 * @param f file to write to
 * @param is indentation space.
 * @param fields fields to write accessor methods for.
 */
void
ToLuaInterfaceGenerator::write_methods_h(FILE *f, std::string /* indent space */ is,
					 std::vector<InterfaceField> fields)
{
  for (vector<InterfaceField>::iterator i = fields.begin(); i != fields.end(); ++i) {

    if ( (i->getLengthValue() > 0) && (i->getType() != "string" ) ) {
      fprintf(f,
	      "%s%s %s%s(int index);\n",
	      is.c_str(),
	      (i->getType() == "byte") ? "unsigned int" : convert_type(i->getPlainAccessType()),
	      ( ((*i).getType() == "bool" ) ? "is_" : ""),
	      (*i).getName().c_str());

      fprintf(f,
	      "%svoid set_%s(unsigned int index, const %s new_%s);\n",
	      is.c_str(), (*i).getName().c_str(),
	      convert_type(i->getPlainAccessType()), i->getName().c_str());
    } else {
      fprintf(f,
	      "%s%s %s%s();\n",
	      is.c_str(), convert_type(i->getAccessType()),
	      ( ((*i).getType() == "bool" ) ? "is_" : ""),
	      (*i).getName().c_str());

      fprintf(f,
	      "%svoid set_%s(const %s new_%s);\n",
	      is.c_str(), (*i).getName().c_str(),
	      convert_type(i->getAccessType()), i->getName().c_str());
    }
    fprintf(f,
	    "%sint maxlenof_%s() const;\n",
	    is.c_str(), (*i).getName().c_str()
	    );
  }
}


/** Write methods to h file.
 * @param f file to write to
 * @param is indentation space.
 * @param fields fields to write accessor methods for.
 * @param pseudo_maps pseudo maps
 */
void
ToLuaInterfaceGenerator::write_methods_h(FILE *f, std::string /* indent space */ is,
					 std::vector<InterfaceField> fields,
					 std::vector<InterfacePseudoMap> pseudo_maps)
{
  write_methods_h(f, is, fields);

  for (vector<InterfacePseudoMap>::iterator i = pseudo_maps.begin(); i != pseudo_maps.end(); ++i) {
    fprintf(f,
	    "%s%s %s(%s key) const;\n"
	    "%svoid set_%s(const %s key, const %s new_value);\n",
	    is.c_str(), convert_type(i->getType()),
	    (*i).getName().c_str(), convert_type(i->getKeyType()),
	    is.c_str(), (*i).getName().c_str(),
	    convert_type(i->getKeyType()), convert_type(i->getType()));
  }
}


/** Write h file.
 * @param f file to write to
 */
void
ToLuaInterfaceGenerator::write_toluaf(FILE *f)
{
  fprintf(f,
	  "$#include <interfaces/%s>\n"
	  "$using namespace fawkes;\n"
	  "namespace fawkes {\n"
	  "class %s : public Interface\n"
	  "{\n",
	  filename_h.c_str(),
	  class_name.c_str());

  write_constants_h(f);
  write_messages_h(f);
  //write_ctor_dtor_h(f, "  ", class_name);
  write_methods_h(f, "  ", data_fields, pseudo_maps);
  write_superclass_h(f);
  fprintf(f, "\n};\n\n}\n");
}


/** Generator cpp and h files.
 */
void
ToLuaInterfaceGenerator::generate()
{
  char timestring[26]; // 26 is mentioned in man asctime_r
  struct tm timestruct;
  time_t t = time(NULL);
  localtime_r(&t, &timestruct);
  asctime_r(&timestruct, timestring);
  gendate = timestring;

  FILE *toluaf;

  toluaf = fopen(string(dir + filename_tolua).c_str(), "w");

  if ( toluaf == NULL ) {
    printf("Cannot open tolua file %s%s\n", dir.c_str(), filename_tolua.c_str());
  }

  write_toluaf(toluaf);

  fclose(toluaf);
}
