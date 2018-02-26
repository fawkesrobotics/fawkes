/***************************************************************************
 *  effect_visitor.h - A static visitor to translate an effect
 *
 *  Created: Tue 31 Oct 2017 12:39:10 CET 12:39
 *  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#ifndef __PLUGINS_CLIPS_PDDL_PARSER_EFFECT_VISITOR_H_
#define __PLUGINS_CLIPS_PDDL_PARSER_EFFECT_VISITOR_H_

#include <pddl_parser/pddl_parser.h>
#include <boost/variant/variant.hpp>
#include <vector>
#include <string>

class EffectToCLIPSFactVisitor
: public boost::static_visitor<std::vector<std::string>>
{
 public:
  EffectToCLIPSFactVisitor(const std::string &pddl_operator, bool positive);
  std::vector<std::string> operator()(pddl_parser::Atom &a) const;
  std::vector<std::string> operator()(pddl_parser::Predicate &p) const;
 private:
  std::string pddl_operator_;
  bool positive_effect_;
};

#endif /* !__PLUGINS_CLIPS_PDDL_PARSER_EFFECT_VISITOR_H_ */
