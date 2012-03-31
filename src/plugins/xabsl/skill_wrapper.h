
/***************************************************************************
 *  skill_wrapper.h - Wrap a skill as XABSL basic behavior
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

#ifndef __PLUGINS_XABSL_SKILL_WRAPPER_H_
#define __PLUGINS_XABSL_SKILL_WRAPPER_H_

#include <XabslEngine/XabslBasicBehavior.h>

#include <map>
#include <list>
#include <utility>
#include <string>

class XabslSkillWrapper : public xabsl::BasicBehavior
{
 public:
  /** Parameter list.
   * Defines the parameters of a skill. It's a list of name/type pairs. The name
   * is the name of the parameter, the type is the value type.
   */
  typedef std::list<std::pair<std::string, std::string> > ParameterList;

  XabslSkillWrapper(const char *name, xabsl::ErrorHandler &error_handler,
		    ParameterList &params);
  ~XabslSkillWrapper();

  virtual void registerParameters();
  virtual void execute();

  const char * name();

  std::string skill_string();

 private:
  bool __execute;

  /// @cond INTERNALS
  class ParameterValueBase
  {
   public:
    virtual ~ParameterValueBase() {}
  };

  template <typename T>
    class ParameterValue : public ParameterValueBase
  {
   public:
    ParameterValue()
    {
      __value = 0;
    }

    T    get_value() const
    {
      return __value;
    }

    T *  get_value_ptr()
    {
      return &__value;
    }

    void set_value(T value)
    {
      __value = value;
    }
   private:
    T __value;
  };
  /// @endcond

  std::map<std::string, ParameterValueBase *> __param_values;
  ParameterList __params;
};


#endif
