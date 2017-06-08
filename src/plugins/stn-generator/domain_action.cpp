
/***************************************************************************
 *  domain_action.cpp - stn-generator
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

#include "domain_action.h"

namespace fawkes {
namespace stn {

DomainAction::DomainAction(std::string name, std::vector<std::string> params, std::vector<Predicate> preconds,
    std::vector<Predicate> effects, int duration,
    std::vector<std::string> cond_breakups, std::vector<std::string> temp_breakups)
{
  name_ = name;
  params_ = params;
  preconds_ = preconds;
  effects_ = effects;
  duration_ = duration;
  cond_breakups_ = cond_breakups;
  temp_breakups_ = temp_breakups;
}

std::ostream&
operator<<(std::ostream &strm, const DomainAction &a)
{
  strm << "DomainAction: " << a.name_ << std::endl;
  strm << "\tParams:";
  for ( auto& param : a.params_ ) {
    strm << " " << param;
  }
  strm << std::endl << "\tPreconditions:" << std::endl;
  for ( auto& pred : a.preconds_ ) {
    strm << "\t\t" << pred;
  }
  strm << std::endl << "\tEffects:" << std::endl;
  for ( auto& eff : a.effects_) {
    strm << "\t\t" << eff;
  }
  strm << std::endl << "\tDuration: " << std::to_string(a.duration_) << std::endl;
  strm << "\tConditional Breakups:";
  for ( auto& breakup : a.cond_breakups_ ) {
    strm << " " << breakup;
  }
  strm << std::endl << "\tTemporal Breakups:";
  for ( auto& breakup : a.temp_breakups_ ) {
    strm << " " << breakup;
  }
  return strm;
}

const std::string
DomainAction::getName()
{
  return name_;
}

const std::vector<std::string>
DomainAction::params()
{
  return params_;
}

StnAction
DomainAction::generateStnAction(std::string name, std::string params)
{
  std::cout << "Generating StnAction " << name << " with params " << params << std::endl;

  std::vector<Predicate> preconds;
  std::vector<Predicate> effects;

  std::istringstream iss (params);
  std::vector<std::string> params_vec{std::istream_iterator<std::string>{iss},
    std::istream_iterator<std::string>{}};
  std::map<std::string,std::string> params_map;
  if ( params_vec.size() != params_.size() ) {
    std::cout << "Param counts differ for DomainAction (" <<std::to_string(params_.size())
      << ") " << name_ << " and StnAction (" << params << ") "
      << name << std::endl;
  }
  for ( size_t i = 0; i < params_vec.size(); i++ ) { 
    // insert with additional question mark, since it is lost during parsing
    params_map.insert( std::pair<std::string,std::string>("?" + params_.at(i), params_vec.at(i)) );
    std::cout << "Inserting " << "?" + params_.at(i) << " with " << params_vec.at(i) << std::endl;
  }
  std::cout << "Applying precods" << std::endl;
  for ( Predicate pred : preconds_) { 
    std::vector<std::string> precond_attr;
    for ( std::string attr : pred.attrs() ) { 
      std::string opt;
      // check if attribute is a constant or variable (latter starting with ?)
      if (attr.find("?") == 0) {
        if ( params_map.find(attr) == params_map.end() ) {
          std::cout << "err, could not find attribute for precondition: " << attr << std::endl;
        }
        opt = params_map.find(attr)->second;
      } else {
        opt = attr;
      }
      precond_attr.push_back(opt);
    }
    preconds.push_back(Predicate(pred.name(), pred.condition(), precond_attr));
  }
  std::cout << "Applying effects" << std::endl;
  for ( Predicate pred : effects_) { 
    std::vector<std::string> effect_attr;
    for ( auto attr : pred.attrs() ) { 
      std::string opt;
      // check if attribute is a constant or variable (latter starting with ?)
      if (attr.find("?") == 0) {
        if ( params_map.find(attr) == params_map.end() ) {
          std::cout << "err, could not find attribute for effect: " << attr << std::endl;
        }
        opt = params_map.find(attr)->second;
      } else {
        opt = attr;
      }
      effect_attr.push_back(opt);
    }
    effects.push_back(Predicate(pred.name(), pred.condition(), effect_attr));
  }
  std::cout << "StnAction built" << std::endl;

  return StnAction(name_, preconds, effects, params, duration_, cond_breakups_, temp_breakups_);
}

}
}
