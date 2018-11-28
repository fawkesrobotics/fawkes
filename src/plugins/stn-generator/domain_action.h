
/***************************************************************************
 *  domain_action.h - stn-generator
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

#ifndef PLUGINS_DOMAIN_ACTION_H_
#define PLUGINS_DOMAIN_ACTION_H_

#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <sstream>
#include <iterator>
#include <algorithm>

#include "predicate.h"
#include "stn_action.h"

namespace fawkes {
namespace stn {

class DomainAction {

 public:
  DomainAction(const std::string& name,
               const std::vector<std::string>& params,
               const std::vector<Predicate>& preconds,
               const std::vector<Predicate>& effects,
               int duration = 0,
               const std::vector<std::string>& cond_breakups = {},
               const std::vector<std::string>& temp_breakups = {});
  virtual ~DomainAction(){ };

  const std::string getName();
  const std::vector<std::string> params();
  StnAction generateStnAction(std::string name, std::string params);

 private:
  friend std::ostream& operator<<(std::ostream&, const DomainAction&);
  std::string name_;
  std::vector<std::string> params_;
  std::vector<Predicate> preconds_;
  std::vector<Predicate> effects_;
  int duration_;
  std::vector<std::string> cond_breakups_;
  std::vector<std::string> temp_breakups_;
};

}
}
#endif
