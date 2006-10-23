 
/***************************************************************************
 *  field.h - Interface generator field representation
 *
 *  Generated: Wed Oct 11 18:09:57 2006
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

#ifndef __INTERFACES_GENERATOR_FIELD_H_
#define __INTERFACES_GENERATOR_FIELD_H_

#include <string>
#include <vector>

class InterfaceField
{
 public:
  InterfaceField();

  void setComment(const std::string &comment);
  void setName(const std::string &name);
  void setType(const std::string &type);
  void setLength(const std::string &length);
  void setFlags(const std::vector<std::string> &flags);
  void setValidFor(const std::string &validfor);
  void setDefaultValue(const std::string &default_value);
  void setBits(const std::string &type);

  void setAttribute(const std::string &attr_name, const std::string &attr_value);

  void valid();

  std::string               getName() const;
  std::string               getComment() const;
  std::string               getType() const;
  std::string               getAccessType() const;
  std::string               getLength() const;
  std::string               getBits() const;
  unsigned int              getNumBits() const;
  std::vector<std::string>  getFlags() const;
  std::string               getValidFor() const;
  std::string               getDefaultValue() const;

  bool operator< (const InterfaceField &f) const;

 private:
  void tokenize(const std::string&   str,
		std::vector<std::string>& tokens,
		const std::string&   delimiters = " ");


  std::string name;
  std::string type;
  std::string comment;
  std::string length;
  std::string bits;
  std::string validfor;
  std::string default_value;
  std::vector<std::string> flags;

  unsigned int bits_val;
};

#endif
