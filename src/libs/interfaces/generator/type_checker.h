 
/***************************************************************************
 *  type_checker.h - Interface generator type checker
 *
 *  Generated: Wed Oct 11 15:35:26 2006
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

#ifndef __INTERFACES_GENERATOR_TYPE_CHECKER_H_
#define __INTERFACES_GENERATOR_TYPE_CHECKED_H_

#define __STDC_LIMIT_MACROS

#include <string>
#include <vector>

#include <interfaces/generator/enum_constant.h>

class InterfaceDataTypeChecker
{
 public:
  static bool validType(const std::string &type, std::vector<InterfaceEnumConstant> *enum_constants = 0);
  static bool validValue(const std::string &type, const std::string &value);
};


#endif
