
/***************************************************************************
 *  stn.h - stn-generator
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

#ifndef __PLUGINS_STN_H
#define __PLUGINS_STN_H

#include <vector>
#include <string>
#include <algorithm>
#include <iterator>
#include <graphviz/gvc.h>

#include "domain_action.h"
#include "stn_action.h"

namespace stn {

class Stn
{
 public:
  Stn(StnAction init);
  virtual ~Stn();

  void add_domain_action(DomainAction action);
  void add_plan_action(std::string name, std::string params);
  void set_initial_state(StnAction action);
  void generate();
  void drawGraph();

 private:
  struct plan_action {
    std::string name;
    std::string params;
  };

  StnAction initial_state_;

  std::vector<DomainAction> domain_actions_;
  std::vector<plan_action> plan_actions_;
  std::vector<StnAction> stn_actions_;

  std::vector<std::pair<StnAction,StnAction>> cond_edges_;
  std::vector<std::pair<StnAction,StnAction>> temp_edges_;

  StnAction findActionById(size_t id);
};

}

#endif
