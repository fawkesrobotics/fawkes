 
/***************************************************************************
 *  enum_constant.h - Interface generator enum constant representation
 *
 *  Generated: Wed Oct 11 19:40:37 2006
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

#ifndef __INTERFACES_GENERATOR_ENUM_CONSTANT_H_
#define __INTERFACES_GENERATOR_ENUM_CONSTANT_H_

#include <string>
#include <vector>
#include <utility>

class InterfaceEnumConstant
{
 public:
  /** Enumeration item. */
  typedef struct {
    std::string name;			///< Name of item.
    std::string comment;		///< Comment for item.
    bool        has_custom_value;	///< True if custom value set.
    int         custom_value;		///< Custom value.
  } EnumItem;

  InterfaceEnumConstant(const std::string &name, const std::string &comment);

  const std::string &  get_name() const;
  const std::string &  get_comment() const;
  const std::vector<EnumItem> &  get_items() const;
  void add_item(std::string name, std::string comment);
  void add_item(std::string name, std::string comment, int value);

 private:

  std::string __name;
  std::string __comment;
  std::vector<EnumItem> __items;
};

#endif
