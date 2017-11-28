
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

/** @class StnAction "stn_action.h"
 * An action representation within an STN.
 */


size_t StnAction::count = 0;

/** Constructor.
 * @param name The name of the new action.
 * @param preconds A Vector of Predicates that must be satisfied to execute the
 * action.
 * @param effects A vector of Predicates that are applied after the action is
 * executed.
 * @param opts The action parameters.
 * @param duration The duration of the action.
 * @param cond_breakups A vector of conditional breakups as strings.
 * @param temp_breakups A vector of temporal breakups as strings.
 */
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

/** Print relevant information about the StnAction.
 * @param strm The stream to print the information to.
 * @param a The action to show the information about.
 */
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

/** Compare two StnActions.
 * @param o The other StnAction.
 * @return True iff the two actions have the same ID.
 */
bool
StnAction::operator==(const StnAction &o)
{
  return id_ == o.id_;
}

/** Compare two StnActions.
 * @param o The other StnAction.
 * @return True iff the two actions have different IDs.
 */
bool
StnAction::operator!=(const StnAction &o)
{
  return id_ != o.id_;
}

/** Get the ID of the action.
 * @return The unique ID.
 */
size_t
StnAction::id()
{
  return id_;
}

/** Get all IDs of this StnAction's conditional actions.
 * @return A vector of IDs.
 */
std::vector<size_t>
StnAction::condActionIds()
{
  std::vector<size_t> ids;
  for ( auto const &kv : cond_actions_) {
    ids.push_back(kv.first);
  }
  return ids;
}

/** Check if the given predicate is a breakup.
 * @param t The type of of breakup to check.
 * @param p The Predicate to check.
 * @return True iff a breakup by the given predicate is possible.
 */
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
    std::cout << "Break because of: " << p << " ";
    switch (t) {
      case EdgeType::CONDITIONAL : std::cout << "Conditional" << std::endl; break;
      case EdgeType::TEMPORAL : std::cout << "Temporal" << std::endl; break;
    }
    return true;
  }
  return false;
}

/** Get a string representation of the StnAction for the graph representation.
 * @return The string describing the StnAction.
 */
std::string
StnAction::genGraphNodeName()
{
  return  "Action ID: " + std::to_string(id_) + "\n"
     + "Name: " + name_ + "\n"
     + "Params: " + opts_;
}

/** Generate an edge label for the graph representation.
 * @param cond_action The ID of the conditional action to represent.
 * @return The string describing the conditional action.
 */
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

/** Generate a temporal edge for the graph representation.
 * @return The string label for the temporal edge.
 */
std::string
StnAction::genTemporalEdgeLabel()
{
  std::string edge_label = "<FONT COLOR=\"blue\">";
  edge_label += std::to_string(duration_);
  edge_label += "</FONT>";
  return edge_label;
}

/** Generate the conditional actions of this StnAction.
 * @param candidate_actions The actions to be considered as conditional actions.
 */
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

/** Get the effects of the StnAction.
 * @return A vector of Predicates that are part of the effect.
 */
std::vector<Predicate>
StnAction::effects()
{
  return effects_;
}

/** Get the name of the StnAction.
 * @return The name as string.
 */
std::string
StnAction::name()
{
  return name_;
}

/** Get the duration of the StnAction.
 * @return The duration.
 */
size_t
StnAction::duration()
{
  return duration_;
}

/** Get the action parameters.
 * @return The parameters as string.
 */
std::string
StnAction::opts()
{
  return opts_;
}

}
}
