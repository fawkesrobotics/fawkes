 
/***************************************************************************
 *  pseudomap.h - Interface generator pseudo map representation
 *
 *  Created: Thu Nov 20 15:06:39 2008
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

#ifndef __INTERFACES_GENERATOR_PSEUDOMAP_H_
#define __INTERFACES_GENERATOR_PSEUDOMAP_H_

#include <interfaces/generator/enum_constant.h>

#include <string>
#include <list>

class InterfacePseudoMap
{
 public:
  /** Reference list. */
  typedef std::list<std::pair<std::string, std::string> > RefList;

  InterfacePseudoMap(std::string name, std::string type,
		     std::string keytype, std::string comment);

  void valid();

  std::string               getName() const;
  std::string               getComment() const;
  std::string               getType() const;
  std::string               getKeyType() const;

  void                      addRef(std::string fieldname, std::string key);

  RefList &                 getRefList();

 private:
  std::string __name;
  std::string __type;
  std::string __comment;
  std::string __keytype;

  RefList __parefs;
};

#endif
