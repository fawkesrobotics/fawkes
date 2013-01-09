
/***************************************************************************
 *  memory.h - Fawkes in-memory configuration
 *
 *  Created: Sat Dec 29 12:15:48 2012
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

#ifndef __CONFIG_MEMORY_H_
#define __CONFIG_MEMORY_H_

#include <config/config.h>
#include <config/yaml.h>

#include <string>
#include <vector>

namespace fawkes {

class YamlConfigurationNode;
class Mutex;

class MemoryConfiguration : public Configuration
{
 public:
  MemoryConfiguration();
  virtual ~MemoryConfiguration();

  virtual void          copy(Configuration *copyconf);

  virtual void          load(const char *file_path);

  virtual bool          exists(const char *path);
  virtual bool          is_float(const char *path);
  virtual bool          is_uint(const char *path);
  virtual bool          is_int(const char *path);
  virtual bool          is_bool(const char *path);
  virtual bool          is_string(const char *path);
  virtual bool          is_list(const char *path);

  virtual bool          is_default(const char *path);

  virtual std::string     get_type(const char *path);
  virtual float           get_float(const char *path);
  virtual unsigned int    get_uint(const char *path);
  virtual int             get_int(const char *path);
  virtual bool            get_bool(const char *path);
  virtual std::string     get_string(const char *path);
  virtual std::vector<float>         get_floats(const char *path);
  virtual std::vector<unsigned int>  get_uints(const char *path);
  virtual std::vector<int>           get_ints(const char *path);
  virtual std::vector<bool>          get_bools(const char *path);
  virtual std::vector<std::string>   get_strings(const char *path);
  virtual ValueIterator * get_value(const char *path);
  virtual std::string     get_comment(const char *path);
  virtual std::string     get_default_comment(const char *path);

  virtual void          set_float(const char *path, float f);
  virtual void          set_uint(const char *path, unsigned int uint);
  virtual void          set_int(const char *path, int i);
  virtual void          set_bool(const char *path, bool b);
  virtual void          set_string(const char *path, std::string &s);
  virtual void          set_string(const char *path, const char *s);
  virtual void          set_floats(const char *path, std::vector<float> &f);
  virtual void          set_uints(const char *path, std::vector<unsigned int> &uint);
  virtual void          set_ints(const char *path, std::vector<int> &i);
  virtual void          set_bools(const char *path, std::vector<bool> &b);
  virtual void          set_strings(const char *path, std::vector<std::string> &s);
  virtual void          set_strings(const char *path, std::vector<const char *> &s);
  virtual void          set_comment(const char *path, std::string &comment);
  virtual void          set_comment(const char *path, const char *comment);

  virtual void          erase(const char *path);

  virtual void          set_default_float(const char *path, float f);
  virtual void          set_default_uint(const char *path, unsigned int uint);
  virtual void          set_default_int(const char *path, int i);
  virtual void          set_default_bool(const char *path, bool b);
  virtual void          set_default_string(const char *path, std::string &s);
  virtual void          set_default_string(const char *path, const char *s);
  virtual void          set_default_comment(const char *path, const char *comment);
  virtual void          set_default_comment(const char *path, std::string &comment);

  virtual void          erase_default(const char *path);

  ValueIterator * iterator();
  ValueIterator * iterator_default();
  ValueIterator * iterator_hostspecific();
  ValueIterator * search(const char *path);

  void lock();
  bool try_lock();
  void unlock();

  virtual void            try_dump();

 private:
  YamlConfigurationNode *  query(const char *path) const;

  YamlConfigurationNode  *root_;

 private:
  Mutex *mutex_;
};


} // end namespace fawkes

#endif
