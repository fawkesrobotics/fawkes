
/***************************************************************************
 *  sqlite.h - Fawkes configuration stored in a SQLite database
 *
 *  Created: Wed Dec 06 17:20:41 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

struct sqlite3;
struct sqlite3_stmt;

namespace fawkes {

class Mutex;

class SQLiteConfiguration : public Configuration
{
 public:
  SQLiteConfiguration(const char *conf_path = NULL);
  virtual ~SQLiteConfiguration();

  virtual void          copy(Configuration *copyconf);

  virtual void          load(const char *filename, const char *defaults_filename,
			     const char *tag = NULL);

  void load(const char *tag = NULL);

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
    virtual bool          valid();
    
    virtual const char *  path();
    virtual const char *  type();
    
    virtual bool          is_float();
    virtual bool          is_uint();
    virtual bool          is_int();
    virtual bool          is_bool();
    virtual bool          is_string();

    virtual bool          is_default();

    virtual float         get_float();
    virtual unsigned int  get_uint();
    virtual int           get_int();
    virtual bool          get_bool();
    virtual std::string   get_string();

    virtual std::string   get_comment();

   private:
    ::sqlite3_stmt *__stmt;
    void *__p;
  };

  ValueIterator * iterator();
  ValueIterator * search(const char *path);

  void lock();
  bool try_lock();
  void unlock();

 private:
  void            init();
  std::string     get_type(const char *table, const char *path);
  bool            exists(const char *sql, const char *path);
  ::sqlite3_stmt *  get_value(const char *type, const char *path);
  ::sqlite3_stmt *  prepare_update(const char *sql, const char *path);
  ::sqlite3_stmt *  prepare_insert_value(const char *sql, const char *type,
				       const char *path);
  void            execute_insert_or_update(sqlite3_stmt *stmt);
  void dump       (::sqlite3 *tdb, const char *dumpfile);
  void import     (::sqlite3 *tdb, const char *dumpfile);
  void merge_default(const char *default_file, const char *default_dump);

 private:
  ::sqlite3 *db;
  const char *conf_path;
  bool opened;
  Mutex *mutex;

  char *__host_file;
  char *__default_file;
  char *__default_dump;
};

} // end namespace fawkes

#endif
