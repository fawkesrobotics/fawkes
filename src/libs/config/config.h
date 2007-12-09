
/***************************************************************************
 *  config.h - Fawkes configuration interface
 *
 *  Created: Mon Dec 04 17:38:32 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __CONFIG_CONFIG_H_
#define __CONFIG_CONFIG_H_

#include <core/exception.h>
#include <config/change_handler.h>
#include <utils/misc/string_compare.h>
#include <string>
#include <list>
#include <map>

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
  CouldNotOpenConfigException(const char *msg);
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
    virtual bool          valid()                                         = 0;
    
    virtual const char *  path()                                          = 0;
    virtual const char *  type()                                          = 0;
    
    virtual bool          is_float()                                      = 0;
    virtual bool          is_uint()                                       = 0;
    virtual bool          is_int()                                        = 0;
    virtual bool          is_bool()                                       = 0;
    virtual bool          is_string()                                     = 0;

    virtual float         get_float()                                     = 0;
    virtual unsigned int  get_uint()                                      = 0;
    virtual int           get_int()                                       = 0;
    virtual bool          get_bool()                                      = 0;
    virtual std::string   get_string()                                    = 0;

    virtual bool          is_default()                                    = 0;
  };

  virtual void          copy(Configuration *copyconf)                     = 0;

  virtual void          add_change_handler(ConfigurationChangeHandler *h);
  virtual void          rem_change_handler(ConfigurationChangeHandler *h);

  virtual void          load(const char *name, const char *defaults_name,
			     const char *tag = NULL)                      = 0;

  virtual void          tag(const char *tag)                              = 0;
  virtual std::list<std::string> tags()                                   = 0;

  virtual bool          exists(const char *path)                          = 0;
  virtual bool          is_float(const char *path)                        = 0;
  virtual bool          is_uint(const char *path)                         = 0;
  virtual bool          is_int(const char *path)                          = 0;
  virtual bool          is_bool(const char *path)                         = 0;
  virtual bool          is_string(const char *path)                       = 0;

  virtual bool          is_default(const char *path)                      = 0;

  virtual float           get_float(const char *path)                     = 0;
  virtual unsigned int    get_uint(const char *path)                      = 0;
  virtual int             get_int(const char *path)                       = 0;
  virtual bool            get_bool(const char *path)                      = 0;
  virtual std::string     get_string(const char *path)                    = 0;
  virtual ValueIterator * get_value(const char *path)                     = 0;

  virtual void          set_float(const char *path,
				  float f)                                = 0;
  virtual void          set_uint(const char *path,
				 unsigned int uint)                       = 0;
  virtual void          set_int(const char *path,
				int i)                                    = 0;
  virtual void          set_bool(const char *path,
				 bool b)                                  = 0;
  virtual void          set_string(const char *path,
				   std::string s)                         = 0;
  virtual void          set_string(const char *path,
				   const char *s)                         = 0;

  virtual void          erase(const char *path)                           = 0;

  virtual void          set_default_float(const char *path, float f)      = 0;
  virtual void          set_default_uint(const char *path,
					 unsigned int uint)               = 0;
  virtual void          set_default_int(const char *path, int i)          = 0;
  virtual void          set_default_bool(const char *path, bool b)        = 0;
  virtual void          set_default_string(const char *path,
					   std::string s)                 = 0;
  virtual void          set_default_string(const char *path,
					   const char *s)                 = 0;

  virtual void          erase_default(const char *path)                   = 0;

  virtual ValueIterator * iterator()                                      = 0;

  virtual ValueIterator * search(const char *path)                        = 0;

  virtual void            lock()                                          = 0;
  virtual bool            try_lock()                                      = 0;
  virtual void            unlock()                                        = 0;

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

  /** Find handlers for given path.
   * @param path path to get handlers for
   * @return list with config change handlers.
   */
  ChangeHandlerList * find_handlers(const char *path);

};

#endif
