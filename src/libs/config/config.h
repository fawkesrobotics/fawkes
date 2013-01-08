
/***************************************************************************
 *  config.h - Fawkes configuration interface
 *
 *  Created: Mon Dec 04 17:38:32 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __CONFIG_CONFIG_H_
#define __CONFIG_CONFIG_H_

#include <core/exception.h>
#include <utils/misc/string_compare.h>
#include <string>
#include <list>
#include <map>
#include <vector>

namespace fawkes {

class ConfigurationChangeHandler;

class ConfigurationException : public Exception
{
 public:
  ConfigurationException(const char *msg);
  ConfigurationException(const char *prefix, const char *msg);
};

class ConfigEntryNotFoundException : public Exception
{
 public:
  ConfigEntryNotFoundException(const char *path);
};

class ConfigTypeMismatchException : public Exception
{
 public:
  ConfigTypeMismatchException(const char *path,
			       const char *actual, const char *requested);
};

class CouldNotOpenConfigException : public Exception
{
 public:
  CouldNotOpenConfigException(const char *format, ...);
};

class Configuration
{
 public:
  virtual ~Configuration() {}

  class ValueIterator
  {
   public:
    virtual ~ValueIterator() {}
    virtual bool          next()                                          = 0;
    virtual bool          valid() const                                   = 0;
    
    virtual const char *  path() const                                    = 0;
    virtual const char *  type() const                                    = 0;
    
    virtual bool          is_float() const                                = 0;
    virtual bool          is_uint() const                                 = 0;
    virtual bool          is_int() const                                  = 0;
    virtual bool          is_bool() const                                 = 0;
    virtual bool          is_string() const                               = 0;
    virtual bool          is_list() const                                 = 0;
    virtual size_t        get_list_size() const                           = 0;

    virtual float         get_float() const                               = 0;
    virtual unsigned int  get_uint() const                                = 0;
    virtual int           get_int() const                                 = 0;
    virtual bool          get_bool() const                                = 0;
    virtual std::string   get_string() const                              = 0;
    virtual std::vector<float>         get_floats() const                 = 0;
    virtual std::vector<unsigned int>  get_uints() const                  = 0;
    virtual std::vector<int>           get_ints() const                   = 0;
    virtual std::vector<bool>          get_bools() const                  = 0;
    virtual std::vector<std::string>   get_strings() const                = 0;
    virtual std::string   get_as_string() const                           = 0;

    virtual std::string   get_comment() const                             = 0;

    virtual bool          is_default() const                              = 0;
  };

  virtual void          copy(Configuration *copyconf)                     = 0;

  virtual void          add_change_handler(ConfigurationChangeHandler *h);
  virtual void          rem_change_handler(ConfigurationChangeHandler *h);

  virtual void          load(const char *file_path)                       = 0;

  virtual bool          exists(const char *path)                          = 0;
  virtual bool          is_float(const char *path)                        = 0;
  virtual bool          is_uint(const char *path)                         = 0;
  virtual bool          is_int(const char *path)                          = 0;
  virtual bool          is_bool(const char *path)                         = 0;
  virtual bool          is_string(const char *path)                       = 0;
  virtual bool          is_list(const char *path)                         = 0;

  virtual bool          is_default(const char *path)                      = 0;

  virtual float           get_float(const char *path)                     = 0;
  virtual unsigned int    get_uint(const char *path)                      = 0;
  virtual int             get_int(const char *path)                       = 0;
  virtual bool            get_bool(const char *path)                      = 0;
  virtual std::string     get_string(const char *path)                    = 0;
  virtual std::vector<float>         get_floats(const char *path)         = 0;
  virtual std::vector<unsigned int>  get_uints(const char *path)          = 0;
  virtual std::vector<int>           get_ints(const char *path)           = 0;
  virtual std::vector<bool>          get_bools(const char *path)          = 0;
  virtual std::vector<std::string>   get_strings(const char *path)        = 0;
  virtual ValueIterator * get_value(const char *path)                     = 0;
  virtual std::string     get_type(const char *path)                      = 0;
  virtual std::string     get_comment(const char *path)                   = 0;
  virtual std::string     get_default_comment(const char *path)           = 0;

  virtual void          set_float(const char *path, float f)              = 0;
  virtual void          set_uint(const char *path, unsigned int uint)     = 0;
  virtual void          set_int(const char *path, int i)                  = 0;
  virtual void          set_bool(const char *path, bool b)                = 0;
  virtual void          set_string(const char *path, std::string &s)      = 0;
  virtual void          set_string(const char *path, const char *s)       = 0;
  virtual void          set_floats(const char *path, std::vector<float> &f)              = 0;
  virtual void          set_uints(const char *path, std::vector<unsigned int> &uint)     = 0;
  virtual void          set_ints(const char *path, std::vector<int> &i)                  = 0;
  virtual void          set_bools(const char *path, std::vector<bool> &b)                = 0;
  virtual void          set_strings(const char *path, std::vector<std::string> &s)      = 0;
  virtual void          set_strings(const char *path, std::vector<const char *> &s)       = 0;
  virtual void          set_comment(const char *path,
				    const char *comment)                  = 0;
  virtual void          set_comment(const char *path,
				    std::string &comment)                 = 0;

  virtual void          erase(const char *path)                           = 0;

  virtual void          set_default_float(const char *path, float f)      = 0;
  virtual void          set_default_uint(const char *path,
					 unsigned int uint)               = 0;
  virtual void          set_default_int(const char *path, int i)          = 0;
  virtual void          set_default_bool(const char *path, bool b)        = 0;
  virtual void          set_default_string(const char *path,
					   std::string &s)                 = 0;
  virtual void          set_default_string(const char *path,
					   const char *s)                 = 0;

  virtual void          set_default_comment(const char *path,
					    const char *comment)          = 0;
  virtual void          set_default_comment(const char *path,
					    std::string &comment)         = 0;

  virtual void          erase_default(const char *path)                   = 0;

  virtual ValueIterator * iterator()                                      = 0;

  virtual ValueIterator * search(const char *path)                        = 0;

  virtual void            lock()                                          = 0;
  virtual bool            try_lock()                                      = 0;
  virtual void            unlock()                                        = 0;

  virtual void            try_dump()                                      = 0;

 protected:
  /** List that contains pointers to ConfigurationChangeHandler */
  typedef std::list<ConfigurationChangeHandler *> ChangeHandlerList;

  /** Multimap string to config change handlers. */
  typedef std::multimap<const char *, ConfigurationChangeHandler *, StringLess >
          ChangeHandlerMultimap;

  /** Config change handler multimap range. */
  typedef std::pair<ChangeHandlerMultimap::iterator,
                    ChangeHandlerMultimap::iterator>
          ChangeHandlerMultimapRange;

  /** Registered change handlers. */
  ChangeHandlerMultimap                  _change_handlers;
  /** Change handler range. */
  ChangeHandlerMultimapRange             _ch_range;

  ChangeHandlerList * find_handlers(const char *path);
  void notify_handlers(const char *path, bool comment_changed = false);

};

} // end namespace fawkes

#endif
