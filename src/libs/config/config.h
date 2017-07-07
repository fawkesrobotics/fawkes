
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
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

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
  virtual float           get_float_or_default(const char *path, const float &default_val);
  virtual unsigned int	  get_uint_or_default(const char *path, const unsigned int &default_val);
  virtual int             get_int_or_default(const char *path, const int &default_val);
  virtual bool			      get_bool_or_default(const char *path, const bool &default_val);
  virtual std::string     get_string_or_default(const char *path, const std::string &default_val);
  virtual std::vector<float>         get_floats(const char *path)         = 0;
  virtual std::vector<unsigned int>  get_uints(const char *path)          = 0;
  virtual std::vector<int>           get_ints(const char *path)           = 0;
  virtual std::vector<bool>          get_bools(const char *path)          = 0;
  virtual std::vector<std::string>   get_strings(const char *path)        = 0;
  virtual std::vector<float>         get_floats_or_defaults(const char *path, const std::vector<float> &default_val);
  virtual std::vector<unsigned int>  get_uints_or_defaults(const char *path, const std::vector<unsigned int> &default_val);
  virtual std::vector<int>           get_ints_or_defaults(const char *path, const std::vector<int> &default_val);
  virtual std::vector<bool>          get_bools_or_defaults(const char *path, const std::vector<bool> &default_val);
  virtual std::vector<std::string>   get_strings_or_defaults(const char *path, const std::vector<std::string> &default_val);
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

  /// @cond CONVENIENCE_METHODS
  virtual bool          exists(const std::string &path)
  { return exists(path.c_str()); }

  virtual bool          is_float(const std::string &path)  { return is_float(path.c_str()); }
  virtual bool          is_uint(const std::string &path)   { return is_uint(path.c_str()); }
  virtual bool          is_int(const std::string &path)    { return is_int(path.c_str()); }
  virtual bool          is_bool(const std::string &path)   { return is_bool(path.c_str()); }
  virtual bool          is_string(const std::string &path) { return is_string(path.c_str()); }
  virtual bool          is_list(const std::string &path)   { return is_list(path.c_str()); }

  virtual bool          is_default(const std::string &path) { return is_default(path.c_str()); }

  virtual float           get_float(const std::string &path) { return get_float(path.c_str()); }
  virtual unsigned int    get_uint(const std::string &path)  { return get_uint(path.c_str()); }
  virtual int             get_int(const std::string &path)   { return get_int(path.c_str()); }
  virtual bool            get_bool(const std::string &path)  { return get_bool(path.c_str()); }
  virtual std::string     get_string(const std::string &path)
  { return get_string(path.c_str()); }
  virtual std::vector<float>         get_floats(const std::string &path)
  { return get_floats(path.c_str()); }
  virtual std::vector<unsigned int>  get_uints(const std::string &path)
  { return get_uints(path.c_str()); }
  virtual std::vector<int>           get_ints(const std::string &path)
  { return get_ints(path.c_str()); }
  virtual std::vector<bool>          get_bools(const std::string &path)
  { return get_bools(path.c_str()); }
  virtual std::vector<std::string>   get_strings(const std::string &path)
  { return get_strings(path.c_str()); }
  virtual ValueIterator * get_value(const std::string &path)
  { return get_value(path.c_str()); }
  virtual std::string     get_type(const std::string &path)
  { return get_type(path.c_str()); }
  virtual std::string     get_comment(const std::string &path)
  { return get_comment(path.c_str()); }
  virtual std::string     get_default_comment(const std::string &path)
  { return get_default_comment(path.c_str()); }

  virtual void          set_float(const std::string &path, float f)
  { set_float(path.c_str(), f); }
  virtual void          set_uint(const std::string &path, unsigned int uint)
  { set_uint(path.c_str(), uint); }
  virtual void          set_int(const std::string &path, int i)
  { set_int(path.c_str(), i); }
  virtual void          set_bool(const std::string &path, bool b)
  { set_bool(path.c_str(), b); }
  virtual void          set_string(const std::string &path, std::string &s)
  { set_string(path.c_str(), s); }
  virtual void          set_string(const std::string &path, const char *s)
  { set_string(path.c_str(), s); }
  virtual void          set_floats(const std::string &path, std::vector<float> &f)
  { set_floats(path.c_str(), f); }
  virtual void          set_uints(const std::string &path, std::vector<unsigned int> &uint)
  { set_uints(path.c_str(), uint); }
  virtual void          set_ints(const std::string &path, std::vector<int> &i)
  { set_ints(path.c_str(), i); }
  virtual void          set_bools(const std::string &path, std::vector<bool> &b)
  { set_bools(path.c_str(), b); }
  virtual void          set_strings(const std::string &path, std::vector<std::string> &s)
  { set_strings(path.c_str(), s); }
  virtual void          set_strings(const std::string &path, std::vector<const char *> &s)
  { set_strings(path.c_str(), s); }
  virtual void          set_comment(const std::string &path, const char *comment)
  { set_comment(path.c_str(), comment); }
  virtual void          set_comment(const std::string &path, std::string &comment)
  { set_comment(path.c_str(), comment); }

  virtual void          erase(const std::string &path) { erase(path.c_str()); }

  virtual void          set_default_float(const std::string &path, float f)
  { set_default_float(path.c_str(), f); }
  virtual void          set_default_uint(const std::string &path, unsigned int uint)
  { set_default_uint(path.c_str(), uint); }
  virtual void          set_default_int(const std::string &path, int i)
  { set_default_int(path.c_str(), i); }
  virtual void          set_default_bool(const std::string &path, bool b)
  { set_default_bool(path.c_str(), b); }
  virtual void          set_default_string(const std::string &path, std::string &s)
  { set_default_string(path.c_str(), s); }
  virtual void          set_default_string(const std::string &path, const char *s)
  { set_default_string(path.c_str(), s); }

  virtual void          set_default_comment(const std::string &path,
					    const char *comment)
  { set_default_comment(path.c_str(), comment); }
  virtual void          set_default_comment(const std::string &path,
					    std::string &comment)
  { set_default_comment(path.c_str(), comment); }

  virtual void          erase_default(const std::string &path)
  { erase_default(path.c_str()); }

  virtual ValueIterator * search(const std::string &path)
  { return search(path.c_str()); }
  /// @endcond

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
