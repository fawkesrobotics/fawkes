
/***************************************************************************
 *  skill_wrapper.cpp - Wrap a skill as XABSL basic behavior
 *
 *  Created: Sun Aug 10 10:22:22 2008
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

#include "skill_wrapper.h"

#include <core/exception.h>
#include <utils/misc/string_conversions.h>

using std::string;

/** @class XabslSkillWrapper "skill_wrapper.h"
 * Xabsl Skill Wrapper.
 * This wraps a Fawkes skill as a basic behavior for Xabsl.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param name name of the skill
 * @param error_handler Xabsl error handler
 * @param params parameters of this skill
 */
XabslSkillWrapper::XabslSkillWrapper(const char *name,
				     xabsl::ErrorHandler &error_handler,
				     ParameterList &params)
  : xabsl::BasicBehavior(name, error_handler)
{
  params_  = params;
  execute_ = false;
}


/** Destructor. */
XabslSkillWrapper::~XabslSkillWrapper()
{
  std::map<std::string, ParameterValueBase *>::iterator i;
  for (i = param_values_.begin(); i != param_values_.end(); ++i) {
    delete i->second;
  }
  param_values_.clear();
}


/** Get name of the skill.
 * @return skill name
 */
const char *
XabslSkillWrapper::name()
{
  return n;
}


/** Register parameters. */
void
XabslSkillWrapper::registerParameters()
{
  for (ParameterList::iterator i = params_.begin(); i != params_.end(); ++i) {
    if ( (i->second == "float") || (i->second == "double") ||
	 (i->second == "int") || (i->second == "unsigned int") ||
	 (i->second == "long int") || (i->second == "unsigned long int") ) {
      ParameterValue<double> *pv = new ParameterValue<double>();
      param_values_[i->first] = pv;
      parameters->registerDecimal((string(n) + "." + i->first).c_str(), *(pv->get_value_ptr()));
    } else if ( i->second == "bool" ) {
       ParameterValue<bool> *pv = new ParameterValue<bool>();
      param_values_[i->first] = pv;
      parameters->registerBoolean((string(n) + "." + i->first).c_str(), *(pv->get_value_ptr()));
    } else {
      throw fawkes::Exception("Unknown parameter type for field %s in skill %s",
			      i->first.c_str(), n);
    }
  }
}


/** Execute skill. */
void
XabslSkillWrapper::execute()
{
  execute_ = true;
}


/** Get skill string for this string.
 * If execution has been ordered with execute() this method will return a skill
 * string generated based on the given skill name and the parameter list.
 * @return skill string if executed, empty string otherwise
 */
std::string
XabslSkillWrapper::skill_string()
{
  if ( execute_ ) {
    execute_ = false;

    std::string rv = std::string(n) + "{";
    std::map<std::string, ParameterValueBase *>::iterator i;
    bool is_first = true;
    for (i = param_values_.begin(); i != param_values_.end(); ++i) {
      if ( is_first ) {
	is_first = false;
      } else {
	rv += ", ";
      }
      ParameterValue<double> *pvd;
      ParameterValue<bool>   *pvb;
      if ( (pvd = dynamic_cast<ParameterValue<double> *>(i->second)) != NULL) {
        rv += i->first + "=" + fawkes::StringConversions::toString(pvd->get_value());
      } else if ( (pvb = dynamic_cast<ParameterValue<bool> *>(i->second)) != NULL) {
	rv += i->first + "=" + fawkes::StringConversions::toString(pvb->get_value());
      } else { 
        throw fawkes::Exception("Unknonw parameter value type");
      }
    }
    rv += "}";
    return rv;
  } else {
    return "";
  }
}
