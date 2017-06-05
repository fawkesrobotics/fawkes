
/***************************************************************************
 *  stn_action.h - stn-generator
 *
 *  Created: Sat May  6 20:16:21 2017
 *  Copyright  2017  Matthias Loebach
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

#ifndef __PLUGINS_STN_ACTION_H
#define __PLUGINS_STN_ACTION_H

#include <string>
#include <vector>
#include <atomic>
#include <map>
#include <iterator>
#include <algorithm>
#include <iostream>

#include "predicate.h"

namespace fawkes {
namespace stn {

enum EdgeType
{
  CONDITIONAL,
  TEMPORAL
};

class StnAction
{

 public:
  StnAction(std::string name, std::vector<Predicate> preconds, std::vector<Predicate> effects, std::string opts,
      size_t duration = 0, std::vector<std::string> cond_breakups = {}, std::vector<std::string> temp_breakups = {});
  StnAction(){ };
  virtual ~StnAction(){ };

  bool operator==(const StnAction &o);
  bool operator!=(const StnAction &o);

  size_t id();
  bool checkForBreakup(EdgeType t, Predicate p);
  std::vector<size_t> condActionIds();
  std::string genGraphNodeName();
  std::string genConditionEdgeLabel(size_t cond_action);
  std::string genTemporalEdgeLabel();
  void genConditionalActions(std::vector<StnAction> candidate_actions);
  std::vector<Predicate> effects();
  std::string name();
  size_t duration();
  std::string opts();


 private:
  friend std::ostream& operator<<(std::ostream&, const StnAction&);
  size_t id_;
  std::string name_;
  std::vector<Predicate> preconds_;
  std::vector<Predicate> effects_;
  std::string opts_;
  size_t duration_;
  std::vector<std::string> cond_breakups_;
  std::vector<std::string> temp_breakups_;
  std::map<size_t, std::pair<std::string, std::vector<Predicate>>> cond_actions_;
  static size_t count;

};
}
}
#endif
