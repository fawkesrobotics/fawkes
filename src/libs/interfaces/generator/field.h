 
/***************************************************************************
 *  field.h - Interface generator field representation
 *
 *  Generated: Wed Oct 11 18:09:57 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __INTERFACES_GENERATOR_FIELD_H_
#define __INTERFACES_GENERATOR_FIELD_H_

#include <string>
#include <vector>
#include <set>

#include <interfaces/generator/enum_constant.h>

class InterfaceField
{
 public:
  InterfaceField(std::vector<InterfaceEnumConstant> *enum_constants = NULL);

  void setComment(const std::string &comment);
  void setName(const std::string &name);
  void setType(const std::string &type);
  bool isEnumType() const;
  void setLength(const std::string &length);
  void setFlags(const std::vector<std::string> &flags);
  void setValidFor(const std::string &validfor);
  void setDefaultValue(const std::string &default_value);

  void setAttribute(const std::string &attr_name, const std::string &attr_value);

  void valid(const std::set<std::string> &reserved_names);

  std::string               getName() const;
  std::string               getComment() const;
  std::string               getType() const;
  std::string               getAccessType() const;
  std::string               getStructType() const;
  std::string               getPlainAccessType() const;
  std::string               getLength() const;
  unsigned int              getLengthValue() const;
  std::vector<std::string>  getFlags() const;
  std::string               getValidFor() const;
  std::string               getDefaultValue() const;

  const std::vector<InterfaceEnumConstant> *  getEnumConstants() const;
  const InterfaceEnumConstant &               getEnumConstant(const std::string &name) const;

  bool operator< (const InterfaceField &f) const;

 private:
  void tokenize(const std::string&   str,
		std::vector<std::string>& tokens,
		const std::string&   delimiters = " ");


  std::string name;
  std::string type;
  bool        is_enum_type;
  std::string comment;
  std::string length;
  unsigned int length_value;
  std::string validfor;
  std::string default_value;
  std::vector<std::string> flags;
  std::vector<InterfaceEnumConstant> *enum_constants;

};

#endif
