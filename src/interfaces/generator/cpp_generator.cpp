 
/***************************************************************************
 *  cpp_generator.cpp - C++ Interface generator
 *
 *  Created: Thu Oct 12 02:01:27 2006
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

#include "cpp_generator.h"
#include "exceptions.h"

#include <utils/misc/string_conversions.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <time.h>
#include <fstream>

using namespace std;


/** @class CppInterfaceGenerator <interfaces/generator/cpp_generator.h>
 * Generator that transforms input from the InterfaceParser into valid
 * C++ classes.
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
CppInterfaceGenerator::CppInterfaceGenerator(std::string directory, std::string interface_name,
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

  filename_cpp = config_basename + ".cpp";
  filename_h   = config_basename + ".h";
  filename_o   = config_basename + ".o";

  if ( interface_name.find("Interface", 0) == string::npos ) {
    // append Interface
    class_name = interface_name + "Interface";
  } else {
    class_name = interface_name;
  }

  deflector = "__INTERFACES_" + fawkes::StringConversions::to_upper(config_basename) + "_H_";
}


/** Destructor */
CppInterfaceGenerator::~CppInterfaceGenerator()
{
}



/** Write optimized struct.
 * Create struct, try align data well, sort fields:
 * 1. unsigned int
 * 2. int
 * 3. unsigned long int
 * 4. long int
 * 5. float
 * 6. double
 * 7. bool
 * 8. byte
 * 8. string
 * @param f file to write to
 * @param name name of struct
 * @param is indentation space
 * @param fields fields for struct
 */
void
CppInterfaceGenerator::write_struct(FILE *f, std::string name, std::string /* indent space */ is,
				    std::vector<InterfaceField> fields)
{

  //stable_sort(fields.begin(), fields.end());

  fprintf(f,
	  "#pragma pack(push,4)\n"
	  "%s/** Internal data storage, do NOT modify! */\n"
	  "%stypedef struct {\n"
	  "%s  int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */\n"
	  "%s  int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */\n", is.c_str(), is.c_str(), is.c_str(), is.c_str());

  for (vector<InterfaceField>::iterator i = fields.begin(); i != fields.end(); ++i) {
    fprintf(f, "%s  %s %s", is.c_str(), (*i).getStructType().c_str(), (*i).getName().c_str());
    if ( (*i).getLength().length() > 0 ) {
      fprintf(f, "[%s]", (*i).getLength().c_str());
    }
    fprintf(f, "; /**< %s */\n", (*i).getComment().c_str());
  }
  
  fprintf(f, "%s} %s;\n"
	  "#pragma pack(pop)\n\n", is.c_str(), name.c_str());
}


/** Write header to file.
 * @param f file to write to
 * @param filename name of file
 */
void
CppInterfaceGenerator::write_header(FILE *f, std::string filename)
{
  fprintf(f,
	  "\n/***************************************************************************\n"
	  " *  %s - Fawkes BlackBoard Interface - %s\n"
	  " *\n"
	  "%s%s%s"
	  " *  Templated created:   Thu Oct 12 10:49:19 2006\n"
	  " *  Copyright  %s  %s\n"
	  " *\n"
	  " ****************************************************************************/\n\n"
	  "/*  This program is free software; you can redistribute it and/or modify\n"
	  " *  it under the terms of the GNU General Public License as published by\n"
	  " *  the Free Software Foundation; either version 2 of the License, or\n"
	  " *  (at your option) any later version. A runtime exception applies to\n"
	  " *  this software (see LICENSE.GPL_WRE file mentioned below for details).\n"
	  " *\n"
	  " *  This program is distributed in the hope that it will be useful,\n"
	  " *  but WITHOUT ANY WARRANTY; without even the implied warranty of\n"
	  " *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n"
	  " *  GNU Library General Public License for more details.\n"
	  " *\n"
	  " *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.\n"
	  " */\n\n",
	  filename.c_str(), class_name.c_str(),
	  (creation_date.length() > 0 ) ? " *  Interface created: " : "",
	  (creation_date.length() > 0 ) ? creation_date.c_str() : "",
	  (creation_date.length() > 0 ) ? "\n" : "",
	  year.c_str(), (author.length() > 0) ? author.c_str() : "AllemaniACs RoboCup Team"
	  );
}


/** Write header deflector.
 * @param f file to write to
 */
void
CppInterfaceGenerator::write_deflector(FILE *f)
{
  fprintf(f, "#ifndef %s\n", deflector.c_str());
  fprintf(f, "#define %s\n\n", deflector.c_str());
}


/** Write cpp file.
 * @param f file to write to
 */
void
CppInterfaceGenerator::write_cpp(FILE *f)
{
  write_header(f, filename_cpp);
  fprintf(f,
	  "#include <interfaces/%s>\n\n"
	  "#include <core/exceptions/software.h>\n\n"
	  "#include <cstring>\n"
	  "#include <cstdlib>\n\n"
	  "namespace fawkes {\n\n"
	  "/** @class %s <interfaces/%s>\n"
	  " * %s Fawkes BlackBoard Interface.\n"
	  " * %s\n"
	  " * @ingroup FawkesInterfaces\n"
	  " */\n\n\n",
	  filename_h.c_str(), class_name.c_str(), filename_h.c_str(),
	  class_name.c_str(), data_comment.c_str());
  write_constants_cpp(f);
  write_ctor_dtor_cpp(f, class_name, "Interface", "", data_fields, messages);
  write_enum_constants_tostring_cpp(f);
  write_methods_cpp(f, class_name, class_name, data_fields, pseudo_maps, "");
  write_basemethods_cpp(f);
  write_messages_cpp(f);

  write_management_funcs_cpp(f);

  fprintf(f, "\n} // end namespace fawkes\n");
}


/** Write management functions.
 * @param f file to write to
 */
void
CppInterfaceGenerator::write_management_funcs_cpp(FILE *f)
{
  fprintf(f,
	  "/// @cond INTERNALS\n"
	  "EXPORT_INTERFACE(%s)\n"
	  "/// @endcond\n\n",
	  class_name.c_str());
}


/** Write constants to cpp file.
 * @param f file to write to
 */
void
CppInterfaceGenerator::write_constants_cpp(FILE *f)
{
  for ( vector<InterfaceConstant>::iterator i = constants.begin(); i != constants.end(); ++i) {
    const char *type_suffix = "";
    if (i->getType() == "uint32_t") {
      type_suffix = "u";
    }
    fprintf(f,
	    "/** %s constant */\n"
	    "const %s %s::%s = %s%s;\n",
	    (*i).getName().c_str(),
	    (*i).getType().c_str(),
	    class_name.c_str(), i->getName().c_str(),
	    i->getValue().c_str(), type_suffix);
  }
  fprintf(f, "\n");
}


/** Write enum constant tostring methods to cpp file.
 * @param f file to write to
 */
void
CppInterfaceGenerator::write_enum_constants_tostring_cpp(FILE *f)
{
  for ( vector<InterfaceEnumConstant>::iterator i = enum_constants.begin(); i != enum_constants.end(); ++i) {
    fprintf(f,
	    "/** Convert %s constant to string.\n"
	    " * @param value value to convert to string\n"
	    " * @return constant value as string.\n"
	    " */\n"
	    "const char *\n"
	    "%s::tostring_%s(%s value) const\n"
	    "{\n"
	    "  switch (value) {\n",
	    i->getName().c_str(), class_name.c_str(), i->getName().c_str(),
	    i->getName().c_str());
    vector< pair<string,string> > items = (*i).getItems();
    vector< pair<string,string> >::iterator j;
    for (j = items.begin(); j != items.end(); ++j) {
      fprintf(f, "  case %s: return \"%s\";\n",
	      j->first.c_str(), j->first.c_str());
    }
    fprintf(f,
	    "  default: return \"UNKNOWN\";\n"
	    "  }\n"
	    "}\n");
  }
}

/** Write constants to h file
 * @param f file to write to
 */
void
CppInterfaceGenerator::write_constants_h(FILE *f)
{
  fprintf(f, "  /* constants */\n");
  for ( vector<InterfaceConstant>::iterator i = constants.begin(); i != constants.end(); ++i) {
    fprintf(f, "  static const %s %s;\n", (*i).getType().c_str(), (*i).getName().c_str());
  }
  fprintf(f, "\n");

  for ( vector<InterfaceEnumConstant>::iterator i = enum_constants.begin(); i != enum_constants.end(); ++i) {
    fprintf(f,
	    "  /** %s */\n"
	    "  typedef enum {\n",
	    (*i).getComment().c_str());
    vector< pair<string,string> > items = (*i).getItems();
    vector< pair<string,string> >::iterator j = items.begin();
    while (j != items.end()) {
      fprintf(f, "    %s /**< %s */", (*j).first.c_str(), (*j).second.c_str());
      ++j;
      if ( j != items.end() ) {
	fprintf(f, ",\n");
      } else {
	fprintf(f, "\n");
      }
    }
    fprintf(f, "  } %s;\n", (*i).getName().c_str());
    fprintf(f, "  const char * tostring_%s(%s value) const;\n\n",
	    i->getName().c_str(), i->getName().c_str());
  }
}


/** Write messages to h file.
 * @param f file to write to
 */
void
CppInterfaceGenerator::write_messages_h(FILE *f)
{
  fprintf(f, "  /* messages */\n");
  for (vector<InterfaceMessage>::iterator i = messages.begin(); i != messages.end(); ++i) {
    fprintf(f, "  class %s : public Message\n"
	    "  {\n", (*i).getName().c_str());

    fprintf(f, "   private:\n");
    write_struct(f, (*i).getName() + "_data_t", "    ", (*i).getFields());
    fprintf(f,
	    "    %s_data_t *data;\n\n",
	    (*i).getName().c_str());

    fprintf(f, "   public:\n");
    write_message_ctor_dtor_h(f, "    ", (*i).getName(), (*i).getFields());
    write_methods_h(f, "    ", (*i).getFields());
    write_message_clone_method_h(f, "    ");
    fprintf(f, "  };\n\n");
  }
  fprintf(f, "  virtual bool message_valid(const Message *message) const;\n");

}


/** Write messages to cpp file.
 * @param f file to write to
 */
void
CppInterfaceGenerator::write_messages_cpp(FILE *f)
{
  fprintf(f, "/* =========== messages =========== */\n");
  for (vector<InterfaceMessage>::iterator i = messages.begin(); i != messages.end(); ++i) {
    fprintf(f,
	    "/** @class %s::%s <interfaces/%s>\n"
	    " * %s Fawkes BlackBoard Interface Message.\n"
	    " * %s\n"
	    " */\n\n\n",
	    class_name.c_str(), (*i).getName().c_str(), filename_h.c_str(),
	    (*i).getName().c_str(), (*i).getComment().c_str());

    write_message_ctor_dtor_cpp(f, (*i).getName(), "Message", class_name + "::",
				(*i).getFields());
    write_methods_cpp(f, class_name, (*i).getName(), (*i).getFields(), class_name + "::", false);
    write_message_clone_method_cpp(f, (class_name + "::" + (*i).getName()).c_str());
  }
  fprintf(f,
	  "/** Check if message is valid and can be enqueued.\n"
	  " * @param message Message to check\n"
	  " */\n"
	  "bool\n"
	  "%s::message_valid(const Message *message) const\n"
	  "{\n", class_name.c_str());
  unsigned int n = 0;
  for (vector<InterfaceMessage>::iterator i = messages.begin(); i != messages.end(); ++i) {
    fprintf(f,
	    "  const %s *m%u = dynamic_cast<const %s *>(message);\n"
	    "  if ( m%u != NULL ) {\n"
	    "    return true;\n"
	    "  }\n",
	    (*i).getName().c_str(), n, (*i).getName().c_str(), n);
    ++n;
  }
  fprintf(f,
	  "  return false;\n"
	  "}\n\n");
}


/** Write create_message() method to cpp file.
 * @param f file to write to
 */
void
CppInterfaceGenerator::write_create_message_method_cpp(FILE *f)
{
  fprintf(f, "/* =========== message create =========== */\n");
  fprintf(f,
	  "Message *\n"
	  "%s::create_message(const char *type) const\n"
	  "{\n", class_name.c_str());

  bool first = true;
  for (vector<InterfaceMessage>::iterator i = messages.begin(); i != messages.end(); ++i) {
    fprintf(f,
	    "  %sif ( strncmp(\"%s\", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {\n"
	    "    return new %s();\n",
	    first ? "" : "} else ", i->getName().c_str(), i->getName().c_str());
    first = false;
  }
  if (first) {
    fprintf(f,
	    "  throw UnknownTypeException(\"The given type '%%s' does not match any known \"\n"
	    "                             \"message type for this interface type.\", type);\n"
	    "}\n\n\n");
  } else {
    fprintf(f,
	    "  } else {\n"
	    "    throw UnknownTypeException(\"The given type '%%s' does not match any known \"\n"
	    "                               \"message type for this interface type.\", type);\n"
	    "  }\n"
	    "}\n\n\n");
  }
}


/** Write copy_value() method to CPP file.
 * @param f file to write to
 */
void
CppInterfaceGenerator::write_copy_value_method_cpp(FILE *f)
{
  fprintf(f,
	  "/** Copy values from other interface.\n"
	  " * @param other other interface to copy values from\n"
	  " */\n"
	  "void\n"
	  "%s::copy_values(const Interface *other)\n"
	  "{\n"
	  "  const %s *oi = dynamic_cast<const %s *>(other);\n"
	  "  if (oi == NULL) {\n"
	  "    throw TypeMismatchException(\"Can only copy values from interface of same type (%%s vs. %%s)\",\n"
	  "                                type(), other->type());\n"
	  "  }\n"
	  "  memcpy(data, oi->data, sizeof(%s_data_t));\n"
	  "}\n\n",
	  class_name.c_str(), class_name.c_str(), class_name.c_str(), class_name.c_str());
}


/** Write enum_tostring() method to CPP file.
 * @param f file to write to
 */
void
CppInterfaceGenerator::write_enum_tostring_method_cpp(FILE *f)
{
  fprintf(f,
	  "const char *\n"
	  "%s::enum_tostring(const char *enumtype, int val) const\n"
	  "{\n", class_name.c_str());
  for ( vector<InterfaceEnumConstant>::iterator i = enum_constants.begin(); i != enum_constants.end(); ++i) {
    fprintf(f,
	    "  if (strcmp(enumtype, \"%s\") == 0) {\n"
	    "    return tostring_%s((%s)val);\n"
	    "  }\n",
	    i->getName().c_str(), i->getName().c_str(), i->getName().c_str());
  }
  fprintf(f,
	  "  throw UnknownTypeException(\"Unknown enum type %%s\", enumtype);\n"
	  "}\n\n");
}


/** Write base methods.
 * @param f file to write to
 */ 
void
CppInterfaceGenerator::write_basemethods_cpp(FILE *f)
{
  write_create_message_method_cpp(f);
  write_copy_value_method_cpp(f);
  write_enum_tostring_method_cpp(f);
}


/** Write constructor and destructor to h file.
 * @param f file to write to
 * @param is indentation space
 * @param classname name of class
 */
void
CppInterfaceGenerator::write_ctor_dtor_h(FILE *f, std::string /* indent space */ is,
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
CppInterfaceGenerator::write_message_ctor_dtor_h(FILE *f, std::string /* indent space */ is,
						 std::string classname,
						 std::vector<InterfaceField> fields)
{
  vector<InterfaceField>::iterator i;

  if ( fields.size() > 0 ) {

    fprintf(f, "%s%s(", is.c_str(), classname.c_str());

    i = fields.begin();
    while (i != fields.end()) {
      fprintf(f, "const %s ini_%s",
	      (*i).getAccessType().c_str(), (*i).getName().c_str());
      ++i;
      if ( i != fields.end() ) {
	fprintf(f, ", ");
      }
    }

    fprintf(f, ");\n");
  }

  write_ctor_dtor_h(f, is, classname);
  fprintf(f, "%s%s(const %s *m);\n", is.c_str(), classname.c_str(), classname.c_str());

}


/** Write message clone method header.
 * @param f file to write to
 * @param is indentation space
 */
void
CppInterfaceGenerator::write_message_clone_method_h(FILE *f, std::string is)
{
  fprintf(f, "%svirtual Message * clone() const;\n", is.c_str());
}


/** Write message clone method.
 * @param f file to write to
 * @param classname name of message class
 */
void
CppInterfaceGenerator::write_message_clone_method_cpp(FILE *f, std::string classname)
{
  fprintf(f,
	  "/** Clone this message.\n"
	  " * Produces a message of the same type as this message and copies the\n"
	  " * data to the new message.\n"
	  " * @return clone of this message\n"
	  " */\n"
	  "Message *\n"
	  "%s::clone() const\n"
	  "{\n"
	  "  return new %s(this);\n"
	  "}\n", classname.c_str(), classname.c_str());
}

/** Write the add_fieldinfo() calls.
 * @param f file to write to
 * @param fields fields to write field info for
 */
void
CppInterfaceGenerator::write_add_fieldinfo_calls(FILE *f, std::vector<InterfaceField> &fields)
{
  std::vector<InterfaceField>::iterator i;
  for (i = fields.begin(); i != fields.end(); ++i) {
    const char *type = "";
    const char *dataptr = "&";
    const char *enumtype = 0;

    if ( i->getType() == "bool" ) {
      type = "BOOL";
    } else if ( i->getType() == "int8" ) {
      type = "INT8";
    } else if ( i->getType() == "uint8" ) {
      type = "UINT8";
    } else if ( i->getType() == "int16" ) {
      type = "INT16";
    } else if ( i->getType() == "uint16" ) {
      type = "UINT16";
    } else if ( i->getType() == "int32" ) {
      type = "INT32";
    } else if ( i->getType() == "uint32" ) {
      type = "UINT32";
    } else if ( i->getType() == "int64" ) {
      type = "INT64";
    } else if ( i->getType() == "uint64" ) {
      type = "UINT64";
    } else if ( i->getType() == "byte" ) {
      type = "BYTE";
    } else if ( i->getType() == "float" ) {
      type = "FLOAT";
    } else if ( i->getType() == "string" ) {
      type = "STRING";
      dataptr = "";
    } else {
      type = "ENUM";
      enumtype = i->getType().c_str();
    }

    fprintf(f, "  add_fieldinfo(IFT_%s, \"%s\", %u, %sdata->%s%s%s%s);\n",
	    type, i->getName().c_str(),
	    (i->getLengthValue() > 0) ? i->getLengthValue() : 1,
	    dataptr, i->getName().c_str(),
	    enumtype ? ", \"" : "",
	    enumtype ? enumtype : "",
	    enumtype ? "\"" : ""
	    );
  }
}


/** Write constructor and destructor to cpp file.
 * @param f file to write to
 * @param classname name of class
 * @param super_class name of base class
 * @param inclusion_prefix Used if class is included in another class.
 * @param fields fields
 * @param messages messages
 */
void
CppInterfaceGenerator::write_ctor_dtor_cpp(FILE *f,
					   std::string classname, std::string super_class,
					   std::string inclusion_prefix,
					   std::vector<InterfaceField> fields,
					   std::vector<InterfaceMessage> messages)
{
  fprintf(f,
	  "/** Constructor */\n"
	  "%s%s::%s() : %s()\n"
	  "{\n",
	  inclusion_prefix.c_str(), classname.c_str(),
	  classname.c_str(), super_class.c_str());

  fprintf(f,
	  "  data_size = sizeof(%s_data_t);\n"
	  "  data_ptr  = malloc(data_size);\n"
	  "  data      = (%s_data_t *)data_ptr;\n"
	  "  data_ts   = (interface_data_ts_t *)data_ptr;\n"
	  "  memset(data_ptr, 0, data_size);\n",
	  classname.c_str(), classname.c_str());

  write_add_fieldinfo_calls(f, fields);

  for (vector<InterfaceMessage>::iterator i = messages.begin(); i != messages.end(); ++i) {
    fprintf(f, "  add_messageinfo(\"%s\");\n", i->getName().c_str());
  }

  fprintf(f, "  unsigned char tmp_hash[] = {");
  for (size_t st = 0; st < hash_size-1; ++st) {
    fprintf(f, "%#02x, ", hash[st]);
  }
  fprintf(f, "%#02x};\n", hash[hash_size-1]);
  fprintf(f, "  set_hash(tmp_hash);\n");

  fprintf(f,
	  "}\n\n"
	  "/** Destructor */\n"
	  "%s%s::~%s()\n"
	  "{\n"
	  "  free(data_ptr);\n"
	  "}\n",
	  inclusion_prefix.c_str(), classname.c_str(), classname.c_str()
	  );
}


/** Write constructor and destructor for message to cpp file.
 * @param f file to write to
 * @param classname name of class
 * @param super_class name of base class
 * @param inclusion_prefix Used if class is included in another class.
 * @param fields vector of data fields of message
 */
void
CppInterfaceGenerator::write_message_ctor_dtor_cpp(FILE *f,
						   std::string classname, std::string super_class,
						   std::string inclusion_prefix,
						   std::vector<InterfaceField> fields)
{
  vector<InterfaceField>::iterator i;

  if ( fields.size() > 0 ) {
    fprintf(f,
	    "/** Constructor with initial values.\n");

    for (i = fields.begin(); i != fields.end(); ++i) {
      fprintf(f, " * @param ini_%s initial value for %s\n",
	      (*i).getName().c_str(), (*i).getName().c_str());
    }

    fprintf(f,
	    " */\n"
	    "%s%s::%s(",
	    inclusion_prefix.c_str(), classname.c_str(), classname.c_str());

    i = fields.begin();
    while (i != fields.end()) {
      fprintf(f, "const %s ini_%s",
	      (*i).getAccessType().c_str(), (*i).getName().c_str());
      ++i;
      if ( i != fields.end() ) {
	fprintf(f, ", ");
      }
    }

    fprintf(f,") : %s(\"%s\")\n"
	    "{\n"
	    "  data_size = sizeof(%s_data_t);\n"
	    "  data_ptr  = malloc(data_size);\n"
	    "  memset(data_ptr, 0, data_size);\n"
	    "  data      = (%s_data_t *)data_ptr;\n"
	    "  data_ts   = (message_data_ts_t *)data_ptr;\n",
	    super_class.c_str(), classname.c_str(), classname.c_str(), classname.c_str());
    
    for (i = fields.begin(); i != fields.end(); ++i) {
      if ( (*i).getType() == "string" ) {
	fprintf(f, "  strncpy(data->%s, ini_%s, %s);\n",
		(*i).getName().c_str(), (*i).getName().c_str(),
		(*i).getLength().c_str());
      } else if (i->getLengthValue() > 1) {
	fprintf(f, "  memcpy(data->%s, ini_%s, sizeof(%s) * %s);\n",
		i->getName().c_str(), i->getName().c_str(),
		i->getPlainAccessType().c_str(), i->getLength().c_str());


      } else {
	fprintf(f, "  data->%s = ini_%s;\n",
		(*i).getName().c_str(), (*i).getName().c_str());
      }
    }

    write_add_fieldinfo_calls(f, fields);

    fprintf(f, "}\n");
  }

  fprintf(f,
	  "/** Constructor */\n"
	  "%s%s::%s() : %s(\"%s\")\n"
	  "{\n",
	  inclusion_prefix.c_str(), classname.c_str(),
	  classname.c_str(), super_class.c_str(), classname.c_str());

  fprintf(f,
	  "  data_size = sizeof(%s_data_t);\n"
	  "  data_ptr  = malloc(data_size);\n"
	  "  memset(data_ptr, 0, data_size);\n"
	  "  data      = (%s_data_t *)data_ptr;\n"
	  "  data_ts   = (message_data_ts_t *)data_ptr;\n",
	  classname.c_str(), classname.c_str());

  write_add_fieldinfo_calls(f, fields);

  fprintf(f,
	  "}\n\n"
	  "/** Destructor */\n"
	  "%s%s::~%s()\n"
	  "{\n"
	  "  free(data_ptr);\n"
	  "}\n\n",
	  inclusion_prefix.c_str(), classname.c_str(), classname.c_str());

  fprintf(f,
	  "/** Copy constructor.\n"
	  " * @param m message to copy from\n"
	  " */\n"
	  "%s%s::%s(const %s *m) : %s(\"%s\")\n"
	  "{\n",
	  inclusion_prefix.c_str(), classname.c_str(), classname.c_str(),
	  classname.c_str(), super_class.c_str(), classname.c_str());

  fprintf(f,
	  "  data_size = m->data_size;\n"
	  "  data_ptr  = malloc(data_size);\n"
	  "  memcpy(data_ptr, m->data_ptr, data_size);\n"
	  "  data      = (%s_data_t *)data_ptr;\n"
	  "  data_ts   = (message_data_ts_t *)data_ptr;\n",
	  classname.c_str());


  fprintf(f, "}\n\n");
}


/** Write methods to cpp file.
 * @param f file to write to
 * @param interface_classname name of the interface class
 * @param classname name of class (can be interface or message)
 * @param fields fields
 * @param inclusion_prefix used if class is included in another class.
 * @param write_data_changed if true writes code that sets the interface's
 * data_changed flag. Set to true for interface methods, false for message
 * methods.
 */
void
CppInterfaceGenerator::write_methods_cpp(FILE *f, std::string interface_classname,
					 std::string classname,
					 std::vector<InterfaceField> fields,
					 std::string inclusion_prefix,
					 bool write_data_changed)
{
  fprintf(f, "/* Methods */\n");
  for (vector<InterfaceField>::iterator i = fields.begin(); i != fields.end(); ++i) {
    fprintf(f,
	    "/** Get %s value.\n"
	    " * %s\n"
	    " * @return %s value\n"
	    " */\n"
	    "%s%s\n"
	    "%s%s::%s%s() const\n"
	    "{\n"
	    "  return data->%s;\n"
	    "}\n\n",
	    (*i).getName().c_str(),
	    (*i).getComment().c_str(),
	    (*i).getName().c_str(),
	    (*i).isEnumType() ? (interface_classname + "::").c_str() : "",
	    (*i).getAccessType().c_str(),
	    inclusion_prefix.c_str(), classname.c_str(), ( ((*i).getType() == "bool" ) ? "is_" : ""), (*i).getName().c_str(),
	    (*i).getName().c_str() );

    if ( (i->getLengthValue() > 0) && (i->getType() != "string") ) {
      fprintf(f,
	      "/** Get %s value at given index.\n"
	      " * %s\n"
	      " * @param index index of value\n"
	      " * @return %s value\n"
	      " * @exception Exception thrown if index is out of bounds\n"
	      " */\n"
	      "%s%s\n"
	      "%s%s::%s%s(unsigned int index) const\n"
	      "{\n"
	      "  if (index > %s) {\n"
	      "    throw Exception(\"Index value %%u out of bounds (0..%s)\", index);\n"
	      "  }\n"
	      "  return data->%s[index];\n"
	      "}\n\n",
	      (*i).getName().c_str(),
	      (*i).getComment().c_str(),
	      (*i).getName().c_str(),
	      (*i).isEnumType() ? (interface_classname + "::").c_str() : "",
	      (*i).getPlainAccessType().c_str(),
	      inclusion_prefix.c_str(), classname.c_str(),
	      ( ((*i).getType() == "bool" ) ? "is_" : ""), (*i).getName().c_str(),
	      i->getLength().c_str(), i->getLength().c_str(),
	      (*i).getName().c_str() );
    }

    fprintf(f,
	    "/** Get maximum length of %s value.\n"
	    " * @return length of %s value, can be length of the array or number of \n"
	    " * maximum number of characters for a string\n"
	    " */\n"
	    "size_t\n"
	    "%s%s::maxlenof_%s() const\n"
	    "{\n"
	    "  return %s;\n"
	    "}\n\n",
	    i->getName().c_str(), i->getName().c_str(), inclusion_prefix.c_str(),
	    classname.c_str(), i->getName().c_str(),
	    i->getLengthValue() > 0 ? i->getLength().c_str() : "1" );

    fprintf(f,
	    "/** Set %s value.\n"
	    " * %s\n"
	    " * @param new_%s new %s value\n"
	    " */\n"
	    "void\n"
	    "%s%s::set_%s(const %s new_%s)\n"
	    "{\n",
	    (*i).getName().c_str(),
	    (*i).getComment().c_str(),	    
	    (*i).getName().c_str(), (*i).getName().c_str(),
	    inclusion_prefix.c_str(), classname.c_str(), (*i).getName().c_str(), (*i).getAccessType().c_str(), (*i).getName().c_str()
	    );
    if ( (*i).getType() == "string" ) {
      fprintf(f,
	      "  strncpy(data->%s, new_%s, sizeof(data->%s));\n",
	      (*i).getName().c_str(), (*i).getName().c_str(), (*i).getName().c_str());
    } else if ( (*i).getLength() != "" ) {
      fprintf(f,
	      "  memcpy(data->%s, new_%s, sizeof(%s) * %s);\n",
	      (*i).getName().c_str(), (*i).getName().c_str(),
	      (*i).getPlainAccessType().c_str(), (*i).getLength().c_str());
    } else {
      fprintf(f,
	      "  data->%s = new_%s;\n",
	      (*i).getName().c_str(), (*i).getName().c_str());
    }
    fprintf(f, "%s}\n\n", write_data_changed ? "  data_changed = true;\n" : "");

    if ( ((*i).getType() != "string") && ((*i).getLengthValue() > 0) ) {
      fprintf(f,
	      "/** Set %s value at given index.\n"
	      " * %s\n"
	      " * @param new_%s new %s value\n"
	      " * @param index index for of the value\n"
	      " */\n"
	      "void\n"
	      "%s%s::set_%s(unsigned int index, const %s new_%s)\n"
	      "{\n"
	      "  if (index > %s) {\n"
	      "    throw Exception(\"Index value %%u out of bounds (0..%s)\", index);\n"
	      "  }\n"
	      "  data->%s[index] = new_%s;\n"
	      "}\n",
	      (*i).getName().c_str(),
	      (*i).getComment().c_str(),	    
	      (*i).getName().c_str(), (*i).getName().c_str(),
	      inclusion_prefix.c_str(), classname.c_str(), (*i).getName().c_str(),
	      (*i).getPlainAccessType().c_str(), i->getName().c_str(),
	      i->getLength().c_str(), i->getLength().c_str(),
	      i->getName().c_str(), i->getName().c_str());
    }
  }
}


/** Write methods to cpp file including pseudo maps.
 * @param f file to write to
 * @param interface_classname name of the interface class
 * @param classname name of class (can be interface or message)
 * @param fields fields
 * @param pseudo_maps pseudo maps
 * @param inclusion_prefix used if class is included in another class.
 */
void
CppInterfaceGenerator::write_methods_cpp(FILE *f, std::string interface_classname,
					 std::string classname,
					 std::vector<InterfaceField> fields,
					 std::vector<InterfacePseudoMap> pseudo_maps,
					 std::string inclusion_prefix)
{
  write_methods_cpp(f, interface_classname, classname, fields,
		    inclusion_prefix, true);

  for (vector<InterfacePseudoMap>::iterator i = pseudo_maps.begin(); i != pseudo_maps.end(); ++i) {
    fprintf(f,
	    "/** Get %s value.\n"
	    " * %s\n"
	    " * @param key key of the value\n"
	    " * @return %s value\n"
	    " */\n"
	    "%s\n"
	    "%s%s::%s(const %s key) const\n"
	    "{\n",
	    (*i).getName().c_str(),
	    (*i).getComment().c_str(),
	    (*i).getName().c_str(),
            (*i).getType().c_str(),
            inclusion_prefix.c_str(), classname.c_str(), (*i).getName().c_str(),
	    (*i).getKeyType().c_str() );

    InterfacePseudoMap::RefList &reflist = i->getRefList();
    InterfacePseudoMap::RefList::iterator paref;
    bool first = true;
    for (paref = reflist.begin(); paref != reflist.end(); ++paref) {
      fprintf(f, "  %sif (key == %s) {\n"
	         "    return data->%s;\n",
	         first ? "" : "} else ",
	      paref->second.c_str(), paref->first.c_str());
      first = false;
    }
    fprintf(f, "  } else {\n"
	       "    throw Exception(\"Invalid key, cannot retrieve value\");\n"
	       "  }\n"
	       "}\n\n");

    fprintf(f,
	    "/** Set %s value.\n"
	    " * %s\n"
	    " * @param key key of the value\n"
	    " * @param new_value new value\n"
	    " */\n"
	    "void\n"
	    "%s%s::set_%s(const %s key, const %s new_value)\n"
	    "{\n",
	    (*i).getName().c_str(),
	    (*i).getComment().c_str(),	    
	    inclusion_prefix.c_str(), classname.c_str(), (*i).getName().c_str(),
	    (*i).getKeyType().c_str(), (*i).getType().c_str());

    first = true;
    for (paref = reflist.begin(); paref != reflist.end(); ++paref) {
      fprintf(f, "  %sif (key == %s) {\n"
	         "    data->%s = new_value;\n",
	         first ? "" : "} else ",
	      paref->second.c_str(), paref->first.c_str());
      first = false;
    }

    fprintf(f, "  }\n"
	       "}\n\n");
  }
}



/** Write methods to h file.
 * @param f file to write to
 * @param is indentation space.
 * @param fields fields to write accessor methods for.
 */
void
CppInterfaceGenerator::write_methods_h(FILE *f, std::string /* indent space */ is,
				       std::vector<InterfaceField> fields)
{
  fprintf(f, "%s/* Methods */\n", is.c_str());
  for (vector<InterfaceField>::iterator i = fields.begin(); i != fields.end(); ++i) {
    fprintf(f,
	    "%s%s %s%s() const;\n",
	    is.c_str(), (*i).getAccessType().c_str(),
	    ( ((*i).getType() == "bool" ) ? "is_" : ""),
	    (*i).getName().c_str());

    if ((i->getLengthValue() > 0) && (i->getType() != "string")) {
      fprintf(f,
	      "%s%s %s%s(unsigned int index) const;\n"
	      "%svoid set_%s(unsigned int index, const %s new_%s);\n",
	      is.c_str(), i->getPlainAccessType().c_str(),
	      ( ((*i).getType() == "bool" ) ? "is_" : ""),
	      (*i).getName().c_str(),
	      is.c_str(), (*i).getName().c_str(),
	      i->getPlainAccessType().c_str(), i->getName().c_str());
    }

    fprintf(f,
	    "%svoid set_%s(const %s new_%s);\n"
	    "%ssize_t maxlenof_%s() const;\n",
	    is.c_str(), (*i).getName().c_str(),
	    i->getAccessType().c_str(), i->getName().c_str(),
	    is.c_str(), i->getName().c_str()
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
CppInterfaceGenerator::write_methods_h(FILE *f, std::string /* indent space */ is,
				       std::vector<InterfaceField> fields,
				       std::vector<InterfacePseudoMap> pseudo_maps)
{
  write_methods_h(f, is, fields);

  for (vector<InterfacePseudoMap>::iterator i = pseudo_maps.begin(); i != pseudo_maps.end(); ++i) {
    fprintf(f,
	    "%s%s %s(%s key) const;\n"
	    "%svoid set_%s(const %s key, const %s new_value);\n",
	    is.c_str(), (*i).getType().c_str(),
	    (*i).getName().c_str(), (*i).getKeyType().c_str(),
	    is.c_str(), (*i).getName().c_str(),
	    i->getKeyType().c_str(), i->getType().c_str());
  }
}


/** Write base methods header entries.
 * @param f file to write to
 * @param is indentation string
 */
void
CppInterfaceGenerator::write_basemethods_h(FILE *f, std::string is)
{
  fprintf(f,
	  "%svirtual Message * create_message(const char *type) const;\n\n"
	  "%svirtual void copy_values(const Interface *other);\n"
	  "%svirtual const char * enum_tostring(const char *enumtype, int val) const;\n",
	  is.c_str(), is.c_str(), is.c_str());
}

/** Write h file.
 * @param f file to write to
 */
void
CppInterfaceGenerator::write_h(FILE *f)
{
  write_header(f, filename_h);
  write_deflector(f);

  fprintf(f,
	  "#include <interface/interface.h>\n"
	  "#include <interface/message.h>\n"
	  "#include <interface/field_iterator.h>\n\n"
	  "namespace fawkes {\n\n"
	  "class %s : public Interface\n"
	  "{\n"
	  " /// @cond INTERNALS\n"
	  " INTERFACE_MGMT_FRIENDS(%s)\n"
	  " /// @endcond\n"
	  " public:\n",
	  class_name.c_str(),
	  class_name.c_str());

  write_constants_h(f);

  fprintf(f, " private:\n");

  write_struct(f, class_name + "_data_t", "  ", data_fields);

  fprintf(f, "  %s_data_t *data;\n"
	  "\n public:\n", class_name.c_str());

  write_messages_h(f);
  fprintf(f, " private:\n");
  write_ctor_dtor_h(f, "  ", class_name);
  fprintf(f, " public:\n");
  write_methods_h(f, "  ", data_fields, pseudo_maps);
  write_basemethods_h(f, "  ");
  fprintf(f, "\n};\n\n} // end namespace fawkes\n\n#endif\n");
}


/** Generator cpp and h files.
 */
void
CppInterfaceGenerator::generate()
{
  char timestring[26]; // 26 is mentioned in man asctime_r
  struct tm timestruct;
  time_t t = time(NULL);
  localtime_r(&t, &timestruct);
  asctime_r(&timestruct, timestring);
  gendate = timestring;

  FILE *cpp;
  FILE *h;

  cpp = fopen(string(dir + filename_cpp).c_str(), "w");
  h   = fopen(string(dir + filename_h).c_str(), "w");

  if ( cpp == NULL ) {
    printf("Cannot open cpp file %s%s\n", dir.c_str(), filename_cpp.c_str());
  }
  if ( h == NULL ) {
    printf("Cannot open h file %s%s\n", dir.c_str(), filename_h.c_str());
  }
  
  write_cpp(cpp);
  write_h(h);

  fclose(cpp);
  fclose(h);
}
