
/***************************************************************************
 *  sqlite.h - Fawkes configuration stored in a SQLite database
 *
 *  Created: Wed Dec 06 17:20:41 2006
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __CONFIG_SQLITE_H_
#define __CONFIG_SQLITE_H_

#include <config/config.h>
#include <utils/system/hostinfo.h>
#include <list>
#include <string>

struct sqlite3;
struct sqlite3_stmt;

namespace fawkes {

class Mutex;

class SQLiteConfiguration : public Configuration
{
 public:
  SQLiteConfiguration();
  SQLiteConfiguration(const char *sysconfdir, const char *userconfdir = NULL);
  virtual ~SQLiteConfiguration();

  virtual void          copy(Configuration *copyconf);

  virtual void          load(const char *filename);

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

  /** Transaction type.
   * See SQLite Documentation for BEGIN TRANSACTION.
   */
  typedef enum {
    TRANSACTION_DEFERRED,	/**< Deferred transaction, lock acquired late. */
    TRANSACTION_IMMEDIATE,	/**< Immediately acquire lock, reading remains possible. */
    TRANSACTION_EXCLUSIVE	/**< Immediately acquire lock, no more reading or writing possible. */
  } transaction_type_t;

  void transaction_begin(transaction_type_t ttype = TRANSACTION_DEFERRED);
  void transaction_commit();
  void transaction_rollback();

 public:
 class SQLiteValueIterator : public Configuration::ValueIterator
  {
    friend class SQLiteConfiguration;
   protected:
    SQLiteValueIterator(::sqlite3_stmt *stmt, void *p = NULL);
   public:
    virtual ~SQLiteValueIterator();
    virtual bool          next();
    virtual bool          valid() const;
    
    virtual const char *  path() const;
    virtual const char *  type() const;
    
    virtual bool          is_float() const;
    virtual bool          is_uint() const;
    virtual bool          is_int() const;
    virtual bool          is_bool() const;
    virtual bool          is_string() const;
    virtual bool          is_list() const;
    virtual size_t        get_list_size() const;

    virtual bool          is_default() const;

    virtual float         get_float() const;
    virtual unsigned int  get_uint() const;
    virtual int           get_int() const;
    virtual bool          get_bool() const;
    virtual std::string   get_string() const;
    virtual std::vector<float>         get_floats() const;
    virtual std::vector<unsigned int>  get_uints() const;
    virtual std::vector<int>           get_ints() const;
    virtual std::vector<bool>          get_bools() const;
    virtual std::vector<std::string>   get_strings() const;

    virtual std::string   get_as_string() const;

    virtual std::string   get_comment() const;

    std::string           get_modtype() const;
    std::string           get_oldvalue() const;

   private:
    ::sqlite3_stmt *__stmt;
    void *__p;
  };

  ValueIterator * iterator();
  ValueIterator * iterator_default();
  ValueIterator * iterator_hostspecific();
  ValueIterator * search(const char *path);

  void lock();
  bool try_lock();
  void unlock();

  SQLiteValueIterator * modified_iterator();

  void try_dump();

 private:
  void            init_dbs();
  std::string     get_type(const char *table, const char *path);
  bool            exists(const char *sql, const char *path);
  ::sqlite3_stmt *  get_value(const char *type, const char *path);
  ::sqlite3_stmt *  prepare_update(const char *sql, const char *path);
  ::sqlite3_stmt *  prepare_insert_value(const char *sql, const char *type,
				       const char *path);
  void execute_insert_or_update(sqlite3_stmt *stmt);
  void dump(::sqlite3 *tdb, const char *dumpfile);
  void import(::sqlite3 *tdb, const char *dumpfile);
  void import_default(const char *default_dump);
  void attach_default(const char *db_file);

 private:
  ::sqlite3 *db;
  bool opened;
  Mutex *mutex;

  char *__sysconfdir;
  char *__userconfdir;
  char *__host_file;
  char *__default_file;
  char *__default_sql;
};

} // end namespace fawkes

#endif
