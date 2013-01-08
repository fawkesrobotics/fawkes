
/***************************************************************************
 *  memory.cpp - Fawkes in-memory configuration
 *
 *  Created: Sat Dec 29 12:20:51 2012
 *  Copyright  2006-2012  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <config/memory.h>

#include "yaml_node.h"

#include <core/threading/mutex.h>
#include <core/exceptions/software.h>

#include <yaml-cpp/exceptions.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class MemoryConfiguration <config/memory.h>
 * In-memory configuration store.
 * @author Tim Niemueller
 */

/** Constructor. */
MemoryConfiguration::MemoryConfiguration()
{
  root_ = new YamlConfigurationNode();
  mutex_ = new Mutex();
}

/** Destructor. */
MemoryConfiguration::~MemoryConfiguration()
{
  delete root_;
  root_ = NULL;

  delete mutex_;
}


void
MemoryConfiguration::load(const char *file_path)
{
}


void
MemoryConfiguration::copy(Configuration *copyconf)
{
  throw NotImplementedException("MemoryConfig does not support copying of a configuration");
}

bool
MemoryConfiguration::exists(const char *path)
{
  try {
    YamlConfigurationNode *n = root_->find(path);
    return ! n->has_children();
  } catch (Exception &e) {
    return false;
  }
}


std::string
MemoryConfiguration::get_type(const char *path)
{
  YamlConfigurationNode *n = root_->find(path);
  if (n->has_children()) {
    throw ConfigEntryNotFoundException(path);
  }

  return YamlConfigurationNode::Type::to_string(n->get_type());
}

std::string
MemoryConfiguration::get_comment(const char *path)
{
  return "";
}


/** Retrieve value casted to given type T.
 * @param root root node of the tree to search
 * @param path path to query
 * @return value casted as desired
 * @throw YAML::ScalarInvalid thrown if value does not exist or is of
 * a different type.
 */
template<typename T>
static inline T
get_value_as(YamlConfigurationNode *root, const char *path)
{
  YamlConfigurationNode *n = root->find(path);
  if (n->has_children()) {
    throw ConfigEntryNotFoundException(path);
  }
  return n->get_value<T>();
}

/** Retrieve value casted to given type T.
 * @param root root node of the tree to search
 * @param path path to query
 * @return value casted as desired
 * @throw YAML::ScalarInvalid thrown if value does not exist or is of
 * a different type.
 */
template<typename T>
static inline std::vector<T>
get_list(YamlConfigurationNode *root, const char *path)
{
  YamlConfigurationNode *n = root->find(path);
  if (n->has_children()) {
    throw ConfigEntryNotFoundException(path);
  }
  return n->get_list<T>();
}


float
MemoryConfiguration::get_float(const char *path)
{
  return get_value_as<float>(root_, path);
}

unsigned int
MemoryConfiguration::get_uint(const char *path)
{
  return get_value_as<unsigned int>(root_, path);
}

int
MemoryConfiguration::get_int(const char *path)
{
  return get_value_as<int>(root_, path);
}

bool
MemoryConfiguration::get_bool(const char *path)
{
  return get_value_as<bool>(root_, path);
}

std::string
MemoryConfiguration::get_string(const char *path)
{
  return get_value_as<std::string>(root_, path);
}


std::vector<float>
MemoryConfiguration::get_floats(const char *path)
{
  return get_list<float>(root_, path);
}


std::vector<unsigned int>
MemoryConfiguration::get_uints(const char *path)
{
  return get_list<unsigned int>(root_, path);
}


std::vector<int>
MemoryConfiguration::get_ints(const char *path)
{
  return get_list<int>(root_, path);
}

std::vector<bool>
MemoryConfiguration::get_bools(const char *path)
{
  return get_list<bool>(root_, path);
}

std::vector<std::string>
MemoryConfiguration::get_strings(const char *path)
{
  return get_list<std::string>(root_, path);
}


/** Check if value is of given type T.
 * @param root root node of the tree to search
 * @param path path to query
 * @return true if value is of desired type, false otherwise
 */
template<typename T>
static inline bool
is_type(YamlConfigurationNode *root, const char *path)
{
  YamlConfigurationNode *n = root->find(path);
  if (n->has_children()) {
    throw ConfigEntryNotFoundException(path);
  }
  return n->is_type<T>();
}


bool
MemoryConfiguration::is_float(const char *path)
{
  return is_type<float>(root_, path);
}

bool
MemoryConfiguration::is_uint(const char *path)
{
  return is_type<unsigned int>(root_, path);
}

bool
MemoryConfiguration::is_int(const char *path)
{
  return is_type<int>(root_, path);
}

bool
MemoryConfiguration::is_bool(const char *path)
{
  return is_type<bool>(root_, path);
}

bool
MemoryConfiguration::is_string(const char *path)
{
  return is_type<std::string>(root_, path);
}


bool
MemoryConfiguration::is_list(const char *path)
{
  YamlConfigurationNode *n = root_->find(path);
  if (n->has_children()) {
    throw ConfigEntryNotFoundException(path);
  }
  return (n->get_type() == YamlConfigurationNode::Type::SEQUENCE);
}


std::string
MemoryConfiguration::get_default_comment(const char *path)
{
  return "";
}

bool
MemoryConfiguration::is_default(const char *path)
{
  try {
    YamlConfigurationNode *n = root_->find(path);
    if (n->has_children()) {
      return false;
    }
    return n->is_default();
  } catch (ConfigEntryNotFoundException &e) {
    return false;
  }
  
  return false;
}


Configuration::ValueIterator *
MemoryConfiguration::get_value(const char *path)
{
  try {
    YamlConfigurationNode *n = root_->find(path);
    if (n->has_children()) {
      return new YamlConfiguration::YamlValueIterator();
    }
    std::map<std::string, YamlConfigurationNode *> nodes;
    nodes[path] = n;
    return new YamlConfiguration::YamlValueIterator(nodes);
  } catch (ConfigEntryNotFoundException &e) {
    return new YamlConfiguration::YamlValueIterator();
  }
}


void
MemoryConfiguration::set_float(const char *path, float f)
{
  root_->set_value(path, f);
  root_->set_default(path, false);
}

void
MemoryConfiguration::set_uint(const char *path, unsigned int uint)
{
  root_->set_value(path, uint);
  root_->set_default(path, false);
}

void
MemoryConfiguration::set_int(const char *path, int i)
{
  root_->set_value(path, i);
  root_->set_default(path, false);
}

void
MemoryConfiguration::set_bool(const char *path, bool b)
{
  root_->set_value(path, b);
  root_->set_default(path, false);
}

void
MemoryConfiguration::set_string(const char *path, const char *s)
{
  root_->set_value(path, std::string(s));
  root_->set_default(path, false);
}


void
MemoryConfiguration::set_string(const char *path, std::string &s)
{
  set_string(path, s.c_str());
}

void
MemoryConfiguration::set_floats(const char *path, std::vector<float> &f)
{
  root_->set_list(path, f);
  root_->set_default(path, false);
}

void
MemoryConfiguration::set_uints(const char *path, std::vector<unsigned int> &u)
{
  root_->set_list(path, u);
  root_->set_default(path, false);
}

void
MemoryConfiguration::set_ints(const char *path, std::vector<int> &i)
{
  root_->set_list(path, i);
  root_->set_default(path, false);
}

void
MemoryConfiguration::set_bools(const char *path, std::vector<bool> &b)
{
  root_->set_list(path, b);
  root_->set_default(path, false);
}

void
MemoryConfiguration::set_strings(const char *path, std::vector<std::string> &s)
{
  root_->set_list(path, s);
  root_->set_default(path, false);
}

void
MemoryConfiguration::set_strings(const char *path, std::vector<const char *> &s)
{
  root_->set_list(path, s);
  root_->set_default(path, false);
}

void
MemoryConfiguration::set_comment(const char *path, const char *comment)
{
}

void
MemoryConfiguration::set_comment(const char *path, std::string &comment)
{
}

void
MemoryConfiguration::erase(const char *path)
{
  root_->erase(path);
}

void
MemoryConfiguration::set_default_float(const char *path, float f)
{
  root_->set_value(path, f);
  root_->set_default(path, true);
}

void
MemoryConfiguration::set_default_uint(const char *path, unsigned int uint)
{
  root_->set_value(path, uint);
  root_->set_default(path, true);
}

void
MemoryConfiguration::set_default_int(const char *path, int i)
{
  root_->set_value(path, i);
  root_->set_default(path, true);
}

void
MemoryConfiguration::set_default_bool(const char *path, bool b)
{
  root_->set_value(path, b);
  root_->set_default(path, true);
}

void
MemoryConfiguration::set_default_string(const char *path,
					const char *s)
{
  root_->set_value(path, s);
  root_->set_default(path, true);
}

void
MemoryConfiguration::set_default_string(const char *path, std::string &s)
{
  set_default_string(path, s.c_str());
}

void
MemoryConfiguration::set_default_comment(const char *path, const char *comment)
{
}

void
MemoryConfiguration::set_default_comment(const char *path, std::string &comment)
{
}


void
MemoryConfiguration::erase_default(const char *path)
{
  root_->erase(path);
}

/** Lock the config.
 * No further changes or queries can be executed on the configuration and will block until
 * the config is unlocked.
 */
void
MemoryConfiguration::lock()
{
  mutex_->lock();
}


/** Try to lock the config.
 * @see Configuration::lock()
 * @return true, if the lock has been aquired, false otherwise
 */
bool
MemoryConfiguration::try_lock()
{
  return mutex_->try_lock();
}

/** Unlock the config.
 * Modifications and queries are possible again.
 */
void
MemoryConfiguration::unlock()
{
  mutex_->unlock();
}


void
MemoryConfiguration::try_dump()
{
}


Configuration::ValueIterator *
MemoryConfiguration::iterator()
{
  std::map<std::string, YamlConfigurationNode *> nodes;
  root_->enum_leafs(nodes);
  return new YamlConfiguration::YamlValueIterator(nodes);
}


/** Get iterator over default values.
 * @return iterator that only mentions default values
 */
Configuration::ValueIterator *
MemoryConfiguration::iterator_default()
{
  std::map<std::string, YamlConfigurationNode *> nodes;
  root_->enum_leafs(nodes);
  std::queue<std::string> delnodes;
  std::map<std::string, YamlConfigurationNode *>::iterator n;
  for (n = nodes.begin(); n != nodes.end(); ++n) {
    if (! n->second->is_default()) {
      delnodes.push(n->first);
    }
  }
  while (!delnodes.empty()) {
    nodes.erase(delnodes.front());
    delnodes.pop();
  }
  return new YamlConfiguration::YamlValueIterator(nodes);
}

/** Get iterator over host-specific values.
 * @return iterator that only mentions host-specific (non-default) values
 */
Configuration::ValueIterator *
MemoryConfiguration::iterator_hostspecific()
{
  std::map<std::string, YamlConfigurationNode *> nodes;
  root_->enum_leafs(nodes);
  std::queue<std::string> delnodes;
  std::map<std::string, YamlConfigurationNode *>::iterator n;
  for (n = nodes.begin(); n != nodes.end(); ++n) {
    if (n->second->is_default()) {
      delnodes.push(n->first);
    }
  }
  while (!delnodes.empty()) {
    nodes.erase(delnodes.front());
    delnodes.pop();
  }
  return new YamlConfiguration::YamlValueIterator(nodes);
}



Configuration::ValueIterator *
MemoryConfiguration::search(const char *path)
{
  std::string tmp_path = path;
  std::string::size_type tl = tmp_path.length();
  if ((tl > 0) && (tmp_path[tl - 1] == '/')) {
    tmp_path.resize(tl - 1);
  }
  try {
    YamlConfigurationNode *n = root_->find(tmp_path.c_str());
    std::map<std::string, YamlConfigurationNode *> nodes;
    n->enum_leafs(nodes, tmp_path);
    return new YamlConfiguration::YamlValueIterator(nodes);
  } catch (Exception &e) {
    return new YamlConfiguration::YamlValueIterator();
  }
}

/** Query node for a specific path.
 * @param path path to retrieve node for
 * @return node representing requested path query result, if the path only
 * consists of collection and path name returns the whole document.
 */
YamlConfigurationNode *
MemoryConfiguration::query(const char *path) const
{
  std::queue<std::string> pel_q = yaml_config::split_to_queue(path);
  return root_->find(pel_q);
}


} // end namespace fawkes
