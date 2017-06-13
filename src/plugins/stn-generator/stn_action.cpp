
/***************************************************************************
 *  stn_action.cpp - stn-generator
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

#include "stn_action.h"

namespace fawkes {
namespace stn {

size_t StnAction::count = 0;
StnAction::StnAction(std::string name, std::vector<Predicate> preconds, std::vector<Predicate> effects, std::string opts,
    size_t duration, std::vector<std::string> cond_breakups, std::vector<std::string> temp_breakups)
{
  id_ = count++;
  name_ = name;
  preconds_ = preconds;
  effects_ = effects;
  opts_ = opts;
  duration_ = duration;
  cond_breakups_ = cond_breakups;
  temp_breakups_ = temp_breakups;
}

std::ostream&
operator<<(std::ostream &strm, const StnAction &a)
{
  strm << "id: " << a.id_ << std::endl << "action: " << a.name_ << "\npreconditions:" << std::endl;
  for ( Predicate p : a.preconds_  ) {
    strm << p;
  }
  strm << "effects:" << std::endl;
  for ( Predicate e : a.effects_ ) {
    strm << e;
  }
  strm << "required actions: ";
  for ( auto const &kv : a.cond_actions_) {
    strm << kv.first << ":" << kv.second.first;
    for ( auto const &p : kv.second.second ) {
      strm << p;
    }
  }
  strm << "duration: " << std::to_string(a.duration_) << std::endl;
  if ( !a.cond_breakups_.empty() ) {
    strm << "conditional breakup conditions: ";
  }
  for ( auto const &p : a.cond_breakups_ ) {
    strm << p;
  }
  if ( !a.temp_breakups_.empty() ) {
    strm << "temporal breakup conditions: ";
  }
  for ( auto const &p : a.temp_breakups_ ) {
    strm << p;
  }
  strm << std::endl << std::endl;

  return strm;
}

bool
StnAction::operator==(const StnAction &o)
{
  return id_ == o.id_;
}

bool
StnAction::operator!=(const StnAction &o)
{
  return id_ != o.id_;
}

size_t
StnAction::id()
{
  return id_;
}

std::vector<size_t>
StnAction::condActionIds()
{
  std::vector<size_t> ids;
  for ( auto const &kv : cond_actions_) {
    ids.push_back(kv.first);
  }
  return ids;
}

bool
StnAction::checkForBreakup(EdgeType t, Predicate p)
{
  std::vector<std::string> *breakups;
  if ( t == EdgeType::CONDITIONAL ) {
    breakups = &cond_breakups_;
  }
  else if ( t == EdgeType::TEMPORAL ) {
    breakups = &temp_breakups_;
  } else {
    throw "Wrong Edge type";
  }

  if ( std::find(breakups->begin(), breakups->end(), p.name()) != breakups->end() ) {
    std::cout << "Break because of: " << p;
    return true;
  }
  return false;
}

std::string
StnAction::genGraphNodeName()
{
  return  "Action ID: " + std::to_string(id_) + "\n"
     + "Name: " + name_ + "\n"
     + "Params: " + opts_;
}

std::string
StnAction::genConditionEdgeLabel(size_t cond_action)
{
  std::string edge_label;
  std::map<size_t, std::pair<std::string, std::vector<Predicate>>>::iterator it = cond_actions_.find(cond_action);
  if ( it == cond_actions_.end() ) return "";
  for ( Predicate p : it->second.second ) {
    if ( p.condition() ) {
      edge_label += "<FONT COLOR=\"darkgreen\">";
    } else {
      edge_label += "<FONT COLOR=\"red\">";
    }
    edge_label += p.name() + ": ";
    for ( const std::string s : p.attrs() ) {
      edge_label += s + " ";
    }
    edge_label += "</FONT>";
  }
  return edge_label;
}

std::string
StnAction::genTemporalEdgeLabel()
{
  std::string edge_label = "<FONT COLOR=\"blue\">";
  edge_label += std::to_string(duration_);
  edge_label += "</FONT>";
  return edge_label;
}

void
StnAction::genConditionalActions(const std::vector<StnAction> candidate_actions)
{ 
  std::vector<Predicate> check_preds = preconds_;
  // iterate backwards to resolve conditions in the correct order
  for ( int i = candidate_actions.size() - 1; i >= 0; i-- ) {
    try {
      for ( Predicate candidate_pred : candidate_actions.at(i).effects_ ) {
        for ( auto pred_it = check_preds.begin(); pred_it != check_preds.end(); ) {
          if ( !checkForBreakup(EdgeType::CONDITIONAL, (*pred_it)) && (*pred_it) == candidate_pred ) {
            std::map<size_t,std::pair<std::string,std::vector<Predicate>>>::iterator it =
              cond_actions_.find(candidate_actions.at(i).id_);
            if ( it == cond_actions_.end() ) {
              cond_actions_.insert(std::map<size_t,
                  std::pair<std::string, std::vector<Predicate>>>::value_type(
                    candidate_actions.at(i).id_,
                    std::make_pair(candidate_actions.at(i).name_, std::vector<Predicate>{(*pred_it)})));
            } else {
              it->second.second.push_back((*pred_it));
            }
            // remove predicate to only take the first (backwards)
            // occurence of a predicate into account _it
            pred_it = check_preds.erase(pred_it);
          } else {
            pred_it++;
          }
        }
      }
    } catch (std::exception& e) {
      std::cout << "ERROR in stn_action: " << e.what() << std::endl;
    }
  }
  
  // erase initial condition if others are present
  /*std::map<size_t, std::pair<std::string, std::vector<Predicate>>>::iterator it = cond_actions_.find(0);
  if ( cond_actions_.size() > 1 && it != cond_actions_.end() ) {
    cond_actions_.erase(it);
  }*/
}

std::vector<Predicate>
StnAction::effects()
{
  return effects_;
}

std::string
StnAction::name()
{
  return name_;
}

size_t
StnAction::duration()
{
  return duration_;
}

std::string
StnAction::opts()
{
  return opts_;
}

}
}
