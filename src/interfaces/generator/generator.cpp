 
/***************************************************************************
 *  generator.cpp - Interface generator
 *
 *  Generated: Thu Oct 12 02:01:27 2006
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth floor, Boston, MA 02111-1307, USA.
 */

#include <interfaces/generator/generator.h>
#include <interfaces/generator/exceptions.h>

#include <utils/misc/string_conversions.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <time.h>
#include <fstream>

using namespace std;


/** @class InterfaceGenerator interfaces/generator/generator.h
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
 */
InterfaceGenerator::InterfaceGenerator(std::string directory, std::string interface_name,
				       std::string config_basename, std::string author,
				       std::string year, std::string creation_date,
				       std::string data_comment,
				       const unsigned char *hash, size_t hash_size)
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
  filename_cpp = config_basename + ".cpp";
  filename_h   = config_basename + ".h";
  filename_o   = config_basename + ".o";

  if ( interface_name.find("Interface", 0) == string::npos ) {
    // append Interface
    class_name = interface_name + "Interface";
  } else {
    class_name = interface_name;
  }

  deflector = "__INTERFACES_" + StringConversions::toUpper(config_basename) + "_H_";
}


/** Destructor */
InterfaceGenerator::~InterfaceGenerator()
{
}


/** Set constants.
 * @param constants parsed constants.
 */
void
InterfaceGenerator::setConstants(const std::vector<InterfaceConstant> &constants)
{
  this->constants = constants;
}


/** Set enum constants.
 * @param enum_constants parsed enum constants
 */
void
InterfaceGenerator::setEnumConstants(const std::vector<InterfaceEnumConstant> &enum_constants)
{
  this->enum_constants = enum_constants;
}


/** Set data fields.
 * @param data_fields parsed data fields.
 */
void
InterfaceGenerator::setDataFields(const std::vector<InterfaceField> &data_fields)
{
  this->data_fields = data_fields;
}


/** Set messages.
 * @param messages parsed messages.
 */
void
InterfaceGenerator::setMessages(const std::vector<InterfaceMessage> &messages)
{
  this->messages = messages;
}


/** Write optimized struct.
 * Create struct, try align data well, sort fields:
 * 1. unsigned int (w/o bit fields)
 * 2. int (w/o bit fields)
 * 3. unsigned long int (w/o bit fields)
 * 4. long int (w/o bit fields)
 * 5. float
 * 6. double
 * 7. bit fields
 * 8. bool
 * 9. char *
 * @param f file to write to
 * @param name name of struct
 * @param is indentation space
 * @param fields fields for struct
 */
void
InterfaceGenerator::write_struct(FILE *f, std::string name, std::string /* indent space */ is,
				 std::vector<InterfaceField> fields)
{

  stable_sort(fields.begin(), fields.end());

  fprintf(f,
	  "%s/** Internal data storage, do NOT modify! */\n"
	  "%stypedef struct {\n", is.c_str(), is.c_str());

  for (vector<InterfaceField>::iterator i = fields.begin(); i != fields.end(); ++i) {
    fprintf(f, "%s  %s %s", is.c_str(), (*i).getType().c_str(), (*i).getName().c_str());
    if ( (*i).getLength().length() > 0 ) {
      fprintf(f, "[%s]", (*i).getLength().c_str());
    } else if ( (*i).getNumBits() > 0 ) {
      fprintf(f, " : %u", (*i).getNumBits());
    }
    fprintf(f, "; /**< %s */\n", (*i).getComment().c_str());
  }

  fprintf(f, "%s} %s;\n\n", is.c_str(), name.c_str());
}


/** Write header to file.
 * @param f file to write to
 * @param filename name of file
 */
void
InterfaceGenerator::write_header(FILE *f, std::string filename)
{
  fprintf(f, "\n/***************************************************************************\n");
  fprintf(f, " *  %s - Fawkes BlackBoard Interface - %s\n", filename.c_str(), class_name.c_str());
  fprintf(f, " *\n");
  if ( creation_date.length() > 0 ) {
    fprintf(f, " *  Interface created: %s\n", creation_date.c_str());
  }
  fprintf(f, " *  Templated created:   Thu Oct 12 10:49:19 2006\n");
  fprintf(f, " *  Copyright  %s  %s\n", year.c_str(),
	  ((author.length() > 0) ? author.c_str() : "AllemaniACs RoboCup Team") );
  fprintf(f, " *\n");
  fprintf(f, " *  $Id$\n");
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


/** Write header deflector.
 * @param f file to write to
 */
void
InterfaceGenerator::write_deflector(FILE *f)
{
  fprintf(f, "#ifndef %s\n", deflector.c_str());
  fprintf(f, "#define %s\n\n", deflector.c_str());
}


/** Write cpp file.
 * @param f file to write to
 */
void
InterfaceGenerator::write_cpp(FILE *f)
{
  write_header(f, filename_cpp);
  fprintf(f,
	  "#include <interfaces/%s>\n\n"
	  "#include <cstring>\n"
	  "#include <cstdlib>\n\n"
	  "/** @class %s interfaces/%s\n"
	  " * %s Fawkes BlackBoard Interface.\n"
	  " * %s\n"
	  " */\n\n\n",
	  filename_h.c_str(), class_name.c_str(), filename_h.c_str(),
	  class_name.c_str(), data_comment.c_str());
  write_constants_cpp(f);
  write_ctor_dtor_cpp(f, class_name, "Interface", "");
  write_methods_cpp(f, class_name, class_name, data_fields, "");
  write_messages_cpp(f);

  write_management_funcs_cpp(f);
}


/** Write management functions.
 * @param f file to write to
 */
void
InterfaceGenerator::write_management_funcs_cpp(FILE *f)
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
InterfaceGenerator::write_constants_cpp(FILE *f)
{
  for ( vector<InterfaceConstant>::iterator i = constants.begin(); i != constants.end(); ++i) {
    fprintf(f,
	    "/** %s constant */\n"
	    "const %s %s::%s = %s;\n",
	    (*i).getName().c_str(),
	    (*i).getType().c_str(),
	    class_name.c_str(), (*i).getName().c_str(), (*i).getValue().c_str());
  }
  fprintf(f, "\n");
}


/** Write constants to h file
 * @param f file to write to
 */
void
InterfaceGenerator::write_constants_h(FILE *f)
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
    fprintf(f, "  } %s;\n\n", (*i).getName().c_str());
  }
}


/** Write messages to h file.
 * @param f file to write to
 */
void
InterfaceGenerator::write_messages_h(FILE *f)
{
  fprintf(f, "  /* messages */\n");
  for (vector<InterfaceMessage>::iterator i = messages.begin(); i != messages.end(); ++i) {
    fprintf(f, "  class %s : public Message\n"
               "  {\n"
	       "   private:\n", (*i).getName().c_str());
    write_struct(f, (*i).getName() + "_data_t", "    ", (*i).getFields());
    fprintf(f,
	    "    %s_data_t *data;\n\n",
	    (*i).getName().c_str());

    fprintf(f, "   public:\n");
    write_message_ctor_dtor_h(f, "    ", (*i).getName(), (*i).getFields());
    write_methods_h(f, "    ", (*i).getFields());

    fprintf(f, "  };\n\n");
  }
  fprintf(f, "  virtual bool message_valid(const Message *message) const;\n");

}


/** Write messages to cpp file.
 * @param f file to write to
 */
void
InterfaceGenerator::write_messages_cpp(FILE *f)
{
  fprintf(f, "/* =========== messages =========== */\n");
  for (vector<InterfaceMessage>::iterator i = messages.begin(); i != messages.end(); ++i) {
    fprintf(f,
	    "/** @class %s::%s interfaces/%s\n"
	    " * %s Fawkes BlackBoard Interface Message.\n"
	    " * %s\n"
	    " */\n\n\n",
	    class_name.c_str(), (*i).getName().c_str(), filename_h.c_str(),
	    (*i).getName().c_str(), (*i).getComment().c_str());

    write_message_ctor_dtor_cpp(f, (*i).getName(), "Message", class_name + "::",
				(*i).getFields());
    write_methods_cpp(f, class_name, (*i).getName(), (*i).getFields(), class_name + "::");

  }
  fprintf(f,
	  "/** Check if message is valid an can be queued.\n"
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


/** Write constructor and destructor to h file.
 * @param f file to write to
 * @param is indentation space
 * @param classname name of class
 */
void
InterfaceGenerator::write_ctor_dtor_h(FILE *f, std::string /* indent space */ is,
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
InterfaceGenerator::write_message_ctor_dtor_h(FILE *f, std::string /* indent space */ is,
					      std::string classname,
					      std::vector<InterfaceField> fields)
{
  vector<InterfaceField>::iterator i;

  if ( fields.size() > 0 ) {

    fprintf(f, "%s%s(", is.c_str(), classname.c_str());

    i = fields.begin();
    while (i != fields.end()) {
      fprintf(f, "%s ini_%s",
	      (*i).getAccessType().c_str(), (*i).getName().c_str());
      ++i;
      if ( i != fields.end() ) {
	fprintf(f, ", ");
      }
    }

    fprintf(f, ");\n");
  }


  write_ctor_dtor_h(f, is, classname);
}


/** Write constructor and destructor to cpp file.
 * @param f file to write to
 * @param classname name of class
 * @param super_class name of base class
 * @param inclusion_prefix Used if class is included in another class.
 */
void
InterfaceGenerator::write_ctor_dtor_cpp(FILE *f,
					std::string classname, std::string super_class,
					std::string inclusion_prefix)
{
  fprintf(f,
	  "/** Constructor */\n"
	  "%s%s::%s() : %s()\n"
	  "{\n"
	  "  data_size = sizeof(%s_data_t);\n"
	  "  data_ptr  = malloc(data_size);\n"
	  "  data      = (%s_data_t *)data_ptr;\n"
	  "  memset(data_ptr, 0, data_size);\n",
	  inclusion_prefix.c_str(), classname.c_str(), classname.c_str(),
	  super_class.c_str(), classname.c_str(), classname.c_str());

  
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
InterfaceGenerator::write_message_ctor_dtor_cpp(FILE *f,
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
      fprintf(f, "%s ini_%s",
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
	    "  data      = (%s_data_t *)data_ptr;\n",
	    super_class.c_str(), classname.c_str(), classname.c_str(), classname.c_str());
    
    for (i = fields.begin(); i != fields.end(); ++i) {
      if ( (*i).getType() == "char" ) {
	fprintf(f, "  strncpy(data->%s, ini_%s, %s);\n",
		(*i).getName().c_str(), (*i).getName().c_str(),
		(*i).getLength().c_str());
      } else {
	fprintf(f, "  data->%s = ini_%s;\n",
		(*i).getName().c_str(), (*i).getName().c_str());
      }
    }

    fprintf(f, "}\n");
  }

  fprintf(f,
	  "/** Constructor */\n"
	  "%s%s::%s() : %s(\"%s\")\n"
	  "{\n"
	  "  data_size = sizeof(%s_data_t);\n"
	  "  data_ptr  = malloc(data_size);\n"
	  "  memset(data_ptr, 0, data_size);\n"
	  "  data      = (%s_data_t *)data_ptr;\n"
	  "}\n\n"
	  "/** Destructor */\n"
	  "%s%s::~%s()\n"
	  "{\n"
	  "  free(data_ptr);\n"
	  "}\n\n",
	  inclusion_prefix.c_str(), classname.c_str(), classname.c_str(),
	  super_class.c_str(), classname.c_str(), classname.c_str(), classname.c_str(),
	  inclusion_prefix.c_str(), classname.c_str(), classname.c_str()
	  );

}


/** Write methods to cpp file.
 * @param f file to write to
 * @param interface_classname name of the interface class
 * @param classname name of class (can be interface or message)
 * @param fields fields
 * @param inclusion_prefix used if class is included in another class.
 */
void
InterfaceGenerator::write_methods_cpp(FILE *f, std::string interface_classname,
				      std::string classname,
				      std::vector<InterfaceField> fields,
				      std::string inclusion_prefix)
{
  fprintf(f, "/* Methods */\n");
  for (vector<InterfaceField>::iterator i = fields.begin(); i != fields.end(); ++i) {
    fprintf(f,
	    "/** Get %s value.\n"
	    " * %s\n"
	    " * @return %s value\n"
	    " */\n"
	    "%s%s\n"
	    "%s%s::%s%s()\n"
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
    if ( (*i).getType() == "char" ) {
      fprintf(f,
	      "  strncpy(data->%s, new_%s, sizeof(data->%s));\n",
	      (*i).getName().c_str(), (*i).getName().c_str(), (*i).getName().c_str());
    } else if ( (*i).getLength() != "" ) {
      fprintf(f,
	      "  memcpy(data->%s, new_%s, sizeof(%s) * %s);\n",
	      (*i).getName().c_str(), (*i).getName().c_str(),
	      (*i).getType().c_str(), (*i).getLength().c_str());
    } else {
      fprintf(f,
	      "  data->%s = new_%s;\n",
	      (*i).getName().c_str(), (*i).getName().c_str());
    }
    fprintf(f, "}\n\n");

  }
}


/** Write methods to h file.
 * @param f file to write to
 * @param is indentation space.
 * @param fields fields to write accessor methods for.
 */
void
InterfaceGenerator::write_methods_h(FILE *f, std::string /* indent space */ is,
				    std::vector<InterfaceField> fields)
{
  fprintf(f, "%s/* Methods */\n", is.c_str());
  for (vector<InterfaceField>::iterator i = fields.begin(); i != fields.end(); ++i) {
    fprintf(f,
	    "%s%s %s%s();\n"
	    "%svoid set_%s(const %s new_%s);\n",
	    is.c_str(), (*i).getAccessType().c_str(),
	    ( ((*i).getType() == "bool" ) ? "is_" : ""),
	    (*i).getName().c_str(),
	    is.c_str(), (*i).getName().c_str(),
	    (*i).getAccessType().c_str(), (*i).getName().c_str()
	    );
  }
}


/** Write h file.
 * @param f file to write to
 */
void
InterfaceGenerator::write_h(FILE *f)
{
  write_header(f, filename_h);
  write_deflector(f);

  fprintf(f,
	  "#include <interface/interface.h>\n"
	  "#include <interface/message.h>\n\n"
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
  write_methods_h(f, "  ", data_fields);
  fprintf(f, "\n};\n\n#endif\n");
}


/** Generator cpp and h files.
 */
void
InterfaceGenerator::generate()
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
