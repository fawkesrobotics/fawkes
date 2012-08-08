
/***************************************************************************
 *  yaml.h - Fawkes configuration stored in one or more YAML files
 *
 *  Created: Wed Aug 01 15:44:33 2012
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

#ifndef __CONFIG_YAML_H_
#define __CONFIG_YAML_H_

#include <config/config.h>

#include <yaml-cpp/yaml.h>
#ifdef USE_REGEX_CPP
// we do not use it atm because it does not work as epxect atm,
// cf. https://bugzilla.redhat.com/show_bug.cgi?id=718711
#  include <regex>
#else
#  include <regex.h>
#endif
#include <memory>
#include <string>
#include <vector>
#include <queue>

namespace fawkes {

class Mutex;

class YamlConfiguration : public Configuration
{
 public:
  YamlConfiguration();
  YamlConfiguration(const char *sysconfdir, const char *userconfdir = NULL);
  virtual ~YamlConfiguration();

  virtual void          copy(Configuration *copyconf);

  virtual void          load(const char *filename, const char *defaults_filename,
			     const char *tag = NULL);

  virtual void          tag(const char *tag);
  virtual std::list<std::string> tags();

  virtual bool          exists(const char *path);
  virtual bool          is_float(const char *path);
  virtual bool          is_uint(const char *path);
  virtual bool          is_int(const char *path);
  virtual bool          is_bool(const char *path);
  virtual bool          is_string(const char *path);

  virtual bool          is_default(const char *path);

  virtual std::string     get_type(const char *path);
  virtual float           get_float(const char *path);
  virtual unsigned int    get_uint(const char *path);
  virtual int             get_int(const char *path);
  virtual bool            get_bool(const char *path);
  virtual std::string     get_string(const char *path);
  virtual ValueIterator * get_value(const char *path);
  virtual std::string     get_comment(const char *path);
  virtual std::string     get_default_comment(const char *path);

  virtual void          set_float(const char *path, float f);
  virtual void          set_uint(const char *path, unsigned int uint);
  virtual void          set_int(const char *path, int i);
  virtual void          set_bool(const char *path, bool b);
  virtual void          set_string(const char *path, std::string &s);
  virtual void          set_string(const char *path, const char *s);
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

  ValueIterator * modified_iterator();

 private:
  class Node;

 public:
  class YamlValueIterator : public Configuration::ValueIterator
 {
  public:
   YamlValueIterator();
   YamlValueIterator(std::map<std::string, Node *> &nodes);

   virtual ~YamlValueIterator() {}
   virtual bool          next();
   virtual bool          valid() const;
    
   virtual const char *  path() const;
   virtual const char *  type() const;
    
   virtual bool          is_float() const;
   virtual bool          is_uint() const;
   virtual bool          is_int() const;
   virtual bool          is_bool() const;
   virtual bool          is_string() const;

   virtual float         get_float() const;
   virtual unsigned int  get_uint() const;
   virtual int           get_int() const;
   virtual bool          get_bool() const;
   virtual std::string   get_string() const;
   virtual std::string   get_as_string() const;

   virtual std::string   get_comment() const;

   virtual bool          is_default() const;

  private:
   bool                                    first_;
   std::map<std::string, Node *>           nodes_;
   std::map<std::string, Node *>::iterator current_;
 };

 private:
  class LoadQueueEntry {
   public:
    LoadQueueEntry(std::string fn, bool im)
    : filename(fn), ignore_missing(im) {}

    std::string filename;
    bool ignore_missing;
  };

  Node *  query(const char *path) const;
  void read_meta_doc(YAML::Node &doc, std::queue<LoadQueueEntry> &load_queue);
  void read_config_doc(const YAML::Node &doc, Node *&node);
  void verify_name(const char *name) const;
  static std::vector<std::string> split(const std::string &s, char delim = '/');
  static std::queue<std::string> split_to_queue(const std::string &s, char delim = '/');

  Node  *root_;

 private:
  Mutex *mutex;

#ifdef USE_REGEX_CPP
  std::regex __path_regex;
  std::regex __yaml_regex;
#else
  regex_t    __path_regex;
  regex_t    __yaml_regex;
#endif

  typedef std::map<std::string, YAML::Node *> DocMap;
  mutable DocMap __documents;

  char *__sysconfdir;
  char *__userconfdir;
  char *__host_file;
  char *__default_file;
  char *__default_sql;
};


} // end namespace fawkes

#endif
