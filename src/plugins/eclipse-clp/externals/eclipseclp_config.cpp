
/***************************************************************************
 *  eclipseclp_config.cpp - config wrapper for Eclipse-CLP
 *
 *  Created: Fri Jan 30 17:17:16 2015 +0100
 *  Copyright  2015  Gesche Gierse
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

#include "eclipseclp_config.h"

#include <eclipseclass.h>
#include <cstring>
#include <typeinfo>

namespace fawkes {

  /** @class fawkes::EclExternalConfig
   * Wrapper class for using the config in the implementation of the external
   * predicates.
   * @author Gesche Gierse
   */

Configuration* EclExternalConfig::m_config = NULL;
EclExternalConfig*  EclExternalConfig::m_instance = NULL;

  /** Constructor. */
  EclExternalConfig::EclExternalConfig() {
    if (m_instance == NULL) {
      m_instance = this;
    }else{
      //throw Exception("There is already an instance of type EclExternalConfig instantiated");
    }
  }

  /** Constructor. */
  EclExternalConfig::EclExternalConfig(Configuration* config) {
    if (m_instance == NULL) {;
      m_instance = this;
      m_config = config;
    }else{
      m_config = config;
      //throw Exception("There is already an instance of type EclExternalConfig instantiated");
    }
  }

  /** Destructor. */
  EclExternalConfig::~EclExternalConfig() {
    delete m_instance;
    //delete m_config;
  }

  /** Creates the initial EclExternalConfig object
   * @param bb pointer to the Configuration to be used
   */
  void EclExternalConfig::create_initial_object(Configuration *bb) {
      m_instance = new EclExternalConfig(bb);
  }


  /** Get the EclExternalConfig instance.
  * @return the instance
  */
  EclExternalConfig* EclExternalConfig::instance()
  {
    if (!m_instance)
    { throw Exception("No instance of type EclExternalConfig instantiated"); }

    return m_instance;
  }


  /** Access the Configuration instance.
   * @return the config instance
   */
  Configuration* EclExternalConfig::config_instance()
  {
    if (!m_config)
    { throw Exception("No instance of type Configuration instantiated"); }

    return m_config;
  }


}


using namespace fawkes;

/* implements get_config_value(Path, Value)
 */
int
p_get_config_value()
{
  Configuration* config = EclExternalConfig::instance()->config_instance();
  char* path;

  if (EC_succeed != EC_arg(1).is_string(&path))
  {
    fprintf(stderr, "p_get_config_value(): no path given\n");
    return EC_fail;
  }

  if (config->is_bool(path))
  {
    if (config->is_list(path))
    {
      std::vector<bool> vec = config->get_bools(path);
      EC_word res = nil();
      for (std::vector<bool>::reverse_iterator it = vec.rbegin() ; it != vec.rend(); ++it)
        res = list(EC_word(*it), res);
      if (EC_succeed != EC_arg(2).unify(res)) {
        fprintf(stderr, "p_get_config_value(): could not bind return value\n");
        return EC_fail;
      }
      return EC_succeed;
    }
    else if (config->get_bool(path)) {
      if (EC_succeed != EC_arg(2).unify(EC_atom((char*) "true"))) {
        fprintf(stderr, "p_get_config_value(): could not bind return value\n");
        return EC_fail;
      }
      return EC_succeed;
    } else {
      if (EC_succeed != EC_arg(2).unify(EC_atom((char*) "false"))) {
        fprintf(stderr, "p_get_config_value(): could not bind return value\n");
        return EC_fail;
      }
      return EC_succeed;
    }
  }
  else if (config->is_int(path))
  {
    if (config->is_list(path))
    {
      std::vector<int> vec = config->get_ints(path);
      EC_word res = nil();
      for (std::vector<int>::reverse_iterator it = vec.rbegin() ; it != vec.rend(); ++it)
        res = list(EC_word((long) *it), res);
      if (EC_succeed != EC_arg(2).unify(res)) {
        fprintf(stderr, "p_get_config_value(): could not bind return value\n");
        return EC_fail;
      }
      return EC_succeed;
    }
    else if (EC_succeed != EC_arg(2).unify(EC_word((long) config->get_int(path)))) {
      fprintf(stderr, "p_get_config_value(): could not bind return value\n");
      return EC_fail;
    }
    return EC_succeed;
  }
  else if (config->is_uint(path))
  {
    if (config->is_list(path))
    {
      std::vector<unsigned int> vec = config->get_uints(path);
      EC_word res = nil();
      for (std::vector<unsigned int>::reverse_iterator it = vec.rend() ; it != vec.rbegin(); --it)
        res = list(EC_word((long) *it), res);
      if (EC_succeed != EC_arg(2).unify(res)) {
        fprintf(stderr, "p_get_config_value(): could not bind return value\n");
        return EC_fail;
      }
      return EC_succeed;
    }
    else if (EC_succeed != EC_arg(2).unify(EC_word((long) config->get_uint(path)))) {
      fprintf(stderr, "p_get_config_value(): could not bind return value\n");
      return EC_fail;
    }
    return EC_succeed;
  }
      
  else if (config->is_float(path))
  {
    if (config->is_list(path))
    {
      std::vector<float> vec = config->get_floats(path);
      EC_word res = nil();
      for (std::vector<float>::reverse_iterator it = vec.rbegin() ; it != vec.rend(); ++it)
        res = list(EC_word((double) *it), res);
      if (EC_succeed != EC_arg(2).unify(res)) {
        fprintf(stderr, "p_get_config_value(): could not bind return value\n");
        return EC_fail;
      }
      return EC_succeed;
    }
    if (EC_succeed != EC_arg(2).unify(EC_word((double) config->get_float(path)))) {
      fprintf(stderr, "p_get_config_value(): could not bind return value\n");
      return EC_fail;
    } 
    return EC_succeed;
  }
  else if (config->is_string(path))
  {
    if (config->is_list(path))
    {
      std::vector<std::string> vec = config->get_strings(path);
      EC_word res = nil();
      for (std::vector<std::string>::reverse_iterator it = vec.rbegin() ; it != vec.rend(); ++it)
        res = list(EC_word( (*it).c_str()), res);
      if (EC_succeed != EC_arg(2).unify(res)) {
        fprintf(stderr, "p_get_config_value(): could not bind return value\n");
        return EC_fail;
      }
      return EC_succeed;
    }
    else if (EC_succeed != EC_arg(2).unify(EC_word(config->get_string(path).c_str()))) {
      fprintf(stderr, "p_get_config_value(): could not bind return value\n");
      return EC_fail;
    }
    return EC_succeed;
  }
  else {
    fprintf(stderr, "p_get_config_value(): could not find type of config value! Type: %s\n", config->get_type(path).c_str() );
    return EC_fail;
  }
  return EC_fail;

}

