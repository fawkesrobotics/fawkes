
/***************************************************************************
 *  sqlite.cpp - Fawkes configuration stored in a SQLite database
 *
 *  Created: Wed Dec 06 17:23:00 2006
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

#include <config/sqlite.h>
#include <core/threading/mutex.h>

#include <sqlite3.h>

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <cstdio>

/* SQLite statements
 */

#define TABLE_HOST_CONFIG "config"
#define TABLE_DEFAULT_CONFIG "defaults.config"
#define TABLE_HOST_TAGGED "tagged_config"

#define SQL_CREATE_TABLE_HOST_CONFIG					\
  "CREATE TABLE IF NOT EXISTS config (\n"				\
  "  path      TEXT NOT NULL,\n"					\
  "  type      TEXT NOT NULL,\n"					\
  "  value     NOT NULL,\n"						\
  "  comment   TEXT,\n"							\
  "  PRIMARY KEY (path)\n"						\
  ")"

#define SQL_CREATE_TABLE_DEFAULT_CONFIG					\
  "CREATE TABLE IF NOT EXISTS defaults.config (\n"			\
  "  path      TEXT NOT NULL,\n"					\
  "  type      TEXT NOT NULL,\n"					\
  "  value     NOT NULL,\n"						\
  "  comment   TEXT,\n"							\
  "  PRIMARY KEY (path)\n"						\
  ")"

#define SQL_CREATE_TABLE_TAGGED_CONFIG					\
  "CREATE TABLE IF NOT EXISTS tagged_config (\n"			\
  "  tag       TEXT NOT NULL,\n"					\
  "  path      TEXT NOT NULL,\n"					\
  "  type      TEXT NOT NULL,\n"					\
  "  value     NOT NULL,\n"						\
  "  comment   TEXT,\n"							\
  "  PRIMARY KEY (tag, path)\n"						\
  ")"

#define SQL_ATTACH_DEFAULTS			\
  "ATTACH DATABASE '%s/%s' AS defaults"

#define SQL_ATTACH_DEFAULTS_ABSOLUTE			\
  "ATTACH DATABASE '%s' AS defaults"

#define SQL_SELECT_VALUE_TYPE						\
  "SELECT type,value FROM config WHERE path=? UNION "			\
  "SELECT type,value FROM defaults.config AS dc "			\
  "WHERE path=? AND NOT EXISTS "					\
  "(SELECT path FROM config WHERE dc.path=path)"

#define SQL_SELECT_COMPLETE						\
  "SELECT * FROM config WHERE path=? UNION "				\
  "SELECT * FROM defaults.config AS dc "				\
  "WHERE path=? AND NOT EXISTS "					\
  "(SELECT path FROM config WHERE dc.path = path)"

#define SQL_SELECT_TYPE							\
  "SELECT type FROM config WHERE path=? UNION "				\
  "SELECT type FROM defaults.config AS dc "				\
  "WHERE path=? AND NOT EXISTS "					\
  "(SELECT path FROM config WHERE dc.path = path)"

#define SQL_UPDATE_VALUE						\
  "UPDATE config SET value=? WHERE path=?"

#define SQL_UPDATE_DEFAULT_VALUE					\
  "UPDATE defaults.config SET value=? WHERE path=?"

#define SQL_INSERT_VALUE						\
  "INSERT INTO config (path, type, value) VALUES (?, ?, ?)"

#define SQL_INSERT_DEFAULT_VALUE					\
  "INSERT INTO defaults.config (path, type, value) VALUES (?, ?, ?)"

#define SQL_SELECT_TAGS							\
  "SELECT tag FROM tagged_config GROUP BY tag"

#define SQL_INSERT_TAG							\
  "INSERT INTO tagged_config "						\
  "(tag, path, type, value, comment) "			\
  "SELECT \"%s\",* FROM config"

#define SQL_SELECT_ALL							\
  "SELECT * FROM config UNION "						\
  "SELECT * FROM defaults.config AS dc WHERE NOT EXISTS "		\
  "(SELECT path FROM config WHERE dc.path = path)"

#define SQL_DELETE_VALUE						\
  "DELETE FROM config WHERE path=?"

#define SQL_DELETE_DEFAULT_VALUE				\
  "DELETE FROM defaults.config WHERE path=?"


/** @class SQLiteConfiguration config/sqlite.h
 * Configuration storage using SQLite.
 * This implementation of the Configuration interface uses SQLite to store the
 * configuration.
 *
 * The configuration uses two databases, one is used to store the host-specific
 * configuration and the other one is used to store the default values. Only the
 * default database is meant to reside under version control.
 *
 * See init() for the structure of the databases. This class strictly serializes
 * all accesses to the database such that only one thread at a time can modify the
 * database.
 */

/** Constructor.
 * @param conf_path Path where the configuration resides, maybe NULL in which case
 * the path name for the base databsae supplied to load() must be absolute path
 * names or relative to the execution directory of the surrounding program.
 */
SQLiteConfiguration::SQLiteConfiguration(const char *conf_path)
{
  this->conf_path = conf_path;
  opened = false;
  mutex = new Mutex();
}


/** Destructor. */
SQLiteConfiguration::~SQLiteConfiguration()
{
  if (opened) {
    opened = false;
    if ( sqlite3_close(db) == SQLITE_BUSY ) {
      printf("Boom, we are dead, database cannot be closed because there are open handles\n");
    }
  }
  delete mutex;
}


/** Initialize the configuration database(s).
 * Initialize databases. If the host-specific database already exists
 * an exception is thrown. You have to delete it before calling init().
 * First the host-specific database is created. It will contain two tables,
 * on is named 'config' and the other one is named 'tagged'. The 'config'
 * table will hold the current configuration for this machine. The 'tagged'
 * table contains the same fields as config with an additional "tag" field.
 * To tag a given revision of the config you give it a name, copy all values
 * over to the 'tagged' table with "tag" set to the desired name.
 *
 * The 'config' table is created with the following schema:
 * @code
 * CREATE TABLE IF NOT EXISTS config (
 *   path      TEXT NOT NULL,
 *   type      TEXT NOT NULL,
 *   value     NOT NULL,
 *   comment   TEXT,
 *   PRIMARY KEY (path)
 * )
 * @endcode
 * If a default database is found the values from this database are copied
 * to the config table.
 * The defaults config database is created with the following structure:
 * @code
 * CREATE TABLE IF NOT EXISTS defaults.config (
 *   path      TEXT NOT NULL,
 *   type      TEXT NOT NULL,
 *   value     NOT NULL,
 *   comment   TEXT,
 *   PRIMARY KEY (path)
 * )
 * @endcode
 *
 * After this the 'tagged' table is created with the following schema:
 * @code
 * CREATE TABLE IF NOT EXISTS tagged_config (
 *   tag       TEXT NOT NULL,
 *   path      TEXT NOT NULL,
 *   type      TEXT NOT NULL,
 *   value     NOT NULL,
 *   comment   TEXT
 *   PRIMARY KEY (tag, path)
 * )
 * @endcode
 *
 * If no default database exists it is created. The database is kept in a file
 * called default.db. It contains a single table called 'config' with the same
 * structure as the 'config' table in the host-specific database.
 */
void
SQLiteConfiguration::init()
{
  char *errmsg;
  if ( (sqlite3_exec(db, SQL_CREATE_TABLE_HOST_CONFIG, NULL, NULL, &errmsg) != SQLITE_OK) ||
       (sqlite3_exec(db, SQL_CREATE_TABLE_DEFAULT_CONFIG, NULL, NULL, &errmsg) != SQLITE_OK) ||
       (sqlite3_exec(db, SQL_CREATE_TABLE_TAGGED_CONFIG, NULL, NULL, &errmsg) != SQLITE_OK) ) {
    CouldNotOpenConfigException ce(sqlite3_errmsg(db));
    sqlite3_close(db);
    throw ce;
  }
}


/** Load configuration.
 * This load the configuration and if requested restores the configuration for the
 * given tag.
 * @param name name of the host-based database. This should be a name of the form
 * hostname.db, where hostname is the unqualified part of the hostname.
 * @param defaults_name name of the default database. Should be defaults.db
 * @param tag optional tag to restore
 */
void
SQLiteConfiguration::load(const char *name, const char *defaults_name,
			  const char *tag)
{
  char *errmsg;
  char *filename;
  char *attach_sql;

  mutex->lock();

  if ( conf_path != NULL ) {
    if ( asprintf(&filename, "%s/%s", conf_path, name) == -1 ) {
      throw CouldNotOpenConfigException("Could not create filename");
    }
    if ( asprintf(&attach_sql, SQL_ATTACH_DEFAULTS, conf_path, defaults_name) == -1 ) {
      free(filename);
      throw CouldNotOpenConfigException("Could not create attachment SQL");
    }
  } else {
    filename = strdup(name);
    if ( asprintf(&attach_sql, SQL_ATTACH_DEFAULTS_ABSOLUTE, defaults_name) == -1 ) {
      free(filename);
      throw CouldNotOpenConfigException("Could not create attachment SQL");
    }
  }
  if ( (sqlite3_open(filename, &db) != SQLITE_OK) ||
       (sqlite3_exec(db, attach_sql, NULL, NULL, &errmsg) != SQLITE_OK) ) {
    CouldNotOpenConfigException ce(sqlite3_errmsg(db));
    free(attach_sql);
    free(filename);
    sqlite3_close(db);
    throw ce;
  }
  free(attach_sql);
  free(filename);

  init();

  mutex->unlock();

  opened = true;
}

/*
 * @fn void Configuration::copy(Configuration *copyconf)
 * Copy all values from the given configuration.
 * All values from the given configuration are copied. Old values are not erased
 * so that the copied values will overwrite existing values, new values are
 * created, but values existent in current config but not in the copie config
 * will remain unchanged.
 * @param copyconf configuration to copy
 */ 
void
SQLiteConfiguration::copy(Configuration *copyconf)
{
  mutex->lock();
  copyconf->lock();
  Configuration::ValueIterator *i = copyconf->iterator();
  while ( i->next() ) {
    if ( i->is_float() ) {
      set_float(i->path(), i->get_float());
    } else if ( i->is_int() ) {
      set_int(i->path(), i->get_int());
    } else if ( i->is_uint() ) {
      set_uint(i->path(), i->get_uint());
    } else if ( i->is_bool() ) {
      set_bool(i->path(), i->get_bool());
    } else if ( i->is_string() ) {
      set_string(i->path(), i->get_string());
    }
  }
  delete i;
  copyconf->unlock();
  mutex->unlock();
}


/** Tag this configuration version.
 * This creates a new tagged version of the current config. The tagged config can be
 * accessed via load().
 * @param tag tag for this version
 */
void
SQLiteConfiguration::tag(const char *tag)
{
  char *insert_sql;
  char *errmsg;

  mutex->lock();

  if ( asprintf(&insert_sql, SQL_INSERT_TAG, tag) == -1 ) {
    mutex->unlock();
    throw ConfigurationException("Could not create insert statement for tagging");
  }

  if (sqlite3_exec(db, insert_sql, NULL, NULL, &errmsg) != SQLITE_OK) {
    ConfigurationException ce("Could not insert tag", sqlite3_errmsg(db));
    free(insert_sql);
    mutex->unlock();
    throw ce;
  }

  free(insert_sql);
  mutex->unlock();
}


/** List of tags.
 * @return list of tags
 */
std::list<std::string>
SQLiteConfiguration::tags()
{
  mutex->lock();
  std::list<std::string> l;
  sqlite3_stmt *stmt;
  const char   *tail;
  if ( sqlite3_prepare(db, SQL_SELECT_TAGS, -1, &stmt, &tail) != SQLITE_OK ) {
    mutex->unlock();
    throw ConfigurationException("get_type: Preparation SQL failed");
  }
  while ( sqlite3_step(stmt) == SQLITE_ROW ) {
    l.push_back((char *)sqlite3_column_text(stmt, 0));
  }
  sqlite3_finalize(stmt);
  mutex->unlock();
  return l;
}


/** Check if a given value exists.
 * @param path path to value
 * @return true if the value exists, false otherwise
 */
bool
SQLiteConfiguration::exists(const char *path)
{
  mutex->lock();
  sqlite3_stmt *stmt;
  const char   *tail;
  bool e;

  if ( sqlite3_prepare(db, SQL_SELECT_TYPE, -1, &stmt, &tail) != SQLITE_OK ) {
    mutex->unlock();
    throw ConfigurationException("exists/prepare", sqlite3_errmsg(db));
  }
  if ( sqlite3_bind_text(stmt, 1, path, -1, NULL) != SQLITE_OK ) {
    mutex->unlock();
    throw ConfigurationException("exists/bind/path", sqlite3_errmsg(db));
  }
  if ( sqlite3_bind_text(stmt, 2, path, -1, NULL) != SQLITE_OK ) {
    mutex->unlock();
    throw ConfigurationException("exists/bind/path", sqlite3_errmsg(db));
  }
  e = ( sqlite3_step(stmt) == SQLITE_ROW );
  sqlite3_finalize(stmt);

  mutex->unlock();
  return e;
}


/** Get type of value.
 * @param path path to value
 * @return string representation of value type
 */
std::string
SQLiteConfiguration::get_type(const char *path)
{
  sqlite3_stmt *stmt;
  const char   *tail;
  std::string   s = "";

  mutex->lock();

  if ( sqlite3_prepare(db, SQL_SELECT_TYPE, -1, &stmt, &tail) != SQLITE_OK ) {
    mutex->unlock();
    throw ConfigurationException("get_type: Preparation SQL failed");
  }
  if ( sqlite3_bind_text(stmt, 1, path, -1, NULL) != SQLITE_OK ) {
    mutex->unlock();
    throw ConfigurationException("get_type: Binding text for path failed (1)");
  }
  if ( sqlite3_bind_text(stmt, 2, path, -1, NULL) != SQLITE_OK ) {
    mutex->unlock();
    throw ConfigurationException("get_type: Binding text for path failed (2)");
  }
  if ( sqlite3_step(stmt) == SQLITE_ROW ) {
    s = (char *)sqlite3_column_text(stmt, 0);
    sqlite3_finalize(stmt);
    mutex->unlock();
    return s;
  } else {
    sqlite3_finalize(stmt);
    mutex->unlock();
    throw ConfigEntryNotFoundException(path);
  }
}


/** Check if a value is of type float
 * @param path path to value
 * @return true if the value exists and is of type float
 */
bool
SQLiteConfiguration::is_float(const char *path)
{
  return (get_type(path) == "float");
}


/** Check if a value is of type unsigned int
 * @param path path to value
 * @return true if the value exists and is of type unsigned int
 */
bool
SQLiteConfiguration::is_uint(const char *path)
{
  return (get_type(path) == "unsigned int");
}


/** Check if a value is of type int
 * @param path path to value
 * @return true if the value exists and is of type int
 */
bool
SQLiteConfiguration::is_int(const char *path)
{
  return (get_type(path) == "int");
}


/** Check if a value is of type bool
 * @param path path to value
 * @return true if the value exists and is of type bool
 */
bool
SQLiteConfiguration::is_bool(const char *path)
{
  return (get_type(path) == "bool");
}


/** Check if a value is of type string
 * @param path path to value
 * @return true if the value exists and is of type string
 */
bool
SQLiteConfiguration::is_string(const char *path)
{
  return (get_type(path) == "string");
}


/** Get value.
 * Get a value from the database.
 * @param path path
 * @param type desired value, NULL to omit type check
 */
sqlite3_stmt *
SQLiteConfiguration::get_value(const char *path,
			       const char *type)
{
  sqlite3_stmt *stmt;
  const char   *tail;

  if ( sqlite3_prepare(db, SQL_SELECT_VALUE_TYPE, -1, &stmt, &tail) != SQLITE_OK ) {
    throw ConfigurationException("get_value/prepare", sqlite3_errmsg(db));
  }
  if ( sqlite3_bind_text(stmt, 1, path, -1, NULL) != SQLITE_OK ) {
    throw ConfigurationException("get_value/bind/path (1)", sqlite3_errmsg(db));
  }
  if ( sqlite3_bind_text(stmt, 2, path, -1, NULL) != SQLITE_OK ) {
    throw ConfigurationException("get_value/bind/path (2)", sqlite3_errmsg(db));
  }

  if ( sqlite3_step(stmt) == SQLITE_ROW ) {
    if ( type == NULL ) {
      // type check omitted
      return stmt;
    } else {
      if (strcmp((char *)sqlite3_column_text(stmt, 0), type) != 0) {
	ConfigTypeMismatchException ce(path, (char *)sqlite3_column_text(stmt, 0), type);
	sqlite3_finalize(stmt);
	throw ce;
      } else {
	return stmt;
      }
    }
  } else {
    sqlite3_finalize(stmt);
    throw ConfigEntryNotFoundException(path);
  }
}


/** Get value from configuration which is of type float
 * @param path path to value
 * @return value
 */
float
SQLiteConfiguration::get_float(const char *path)
{
  sqlite3_stmt *stmt;
  mutex->lock();
  try {
    stmt = get_value(path, "float");
    float f = (float)sqlite3_column_double(stmt, 1);
    sqlite3_finalize(stmt);
    mutex->unlock();
    return f;
  } catch (Exception &e) {
    // we can't handle
    mutex->unlock();
    throw;
  }
}


/** Get value from configuration which is of type unsigned int
 * @param path path to value
 * @return value
 */
unsigned int
SQLiteConfiguration::get_uint(const char *path)
{
  sqlite3_stmt *stmt;
  mutex->lock();
  try {
    stmt = get_value(path, "unsigned int");
    int i = sqlite3_column_int(stmt, 1);
    sqlite3_finalize(stmt);
    if ( i < 0 ) {
      mutex->unlock();
      throw ConfigTypeMismatchException(path, "int", "unsigned int");
    }
    mutex->unlock();
    return i;
  } catch (Exception &e) {
    // we can't handle
    mutex->unlock();
    throw;
  }
}


/** Get value from configuration which is of type int
 * @param path path to value
 * @return value
 */
int
SQLiteConfiguration::get_int(const char *path)
{
  sqlite3_stmt *stmt;
  mutex->lock();
  try {
    stmt = get_value(path, "int");
    int i = sqlite3_column_int(stmt, 1);
    sqlite3_finalize(stmt);
    mutex->unlock();
    return i;
  } catch (Exception &e) {
    // we can't handle
    mutex->unlock();
    throw;
  }
}


/** Get value from configuration which is of type bool
 * @param path path to value
 * @return value
 */
bool
SQLiteConfiguration::get_bool(const char *path)
{
  sqlite3_stmt *stmt;
  mutex->lock();
  try {
    stmt = get_value(path, "bool");
    int i = sqlite3_column_int(stmt, 1);
    sqlite3_finalize(stmt);
    mutex->unlock();
    return (i != 0);
  } catch (Exception &e) {
    // we can't handle
    mutex->unlock();
    throw;
  }
}

/** Get value from configuration which is of type string
 * @param path path to value
 * @return value
 */
std::string
SQLiteConfiguration::get_string(const char *path)
{
  sqlite3_stmt *stmt;
  mutex->lock();
  try {
    stmt = get_value(path, "string");
    const char *c = (char *)sqlite3_column_text(stmt, 1);
    std::string rv = c;
    sqlite3_finalize(stmt);
    mutex->unlock();
    return rv;
  } catch (Exception &e) {
    // we can't handle
    e.append("SQLiteConfiguration::get_string: Fetching %s failed.", path);
    mutex->unlock();
    throw;
  }
}


/** Get value from configuration.
 * @param path path to value
 * @return value iterator for just this one value, maybe invalid if value does not
 * exists.
 */
Configuration::ValueIterator *
SQLiteConfiguration::get_value(const char *path)
{
  sqlite3_stmt *stmt;
  const char   *tail;

  if ( sqlite3_prepare(db, SQL_SELECT_COMPLETE, -1, &stmt, &tail) != SQLITE_OK ) {
    throw ConfigurationException("get_value/prepare", sqlite3_errmsg(db));
  }
  if ( sqlite3_bind_text(stmt, 1, path, -1, NULL) != SQLITE_OK ) {
    throw ConfigurationException("get_value/bind/path (1)", sqlite3_errmsg(db));
  }
  if ( sqlite3_bind_text(stmt, 2, path, -1, NULL) != SQLITE_OK ) {
    throw ConfigurationException("get_value/bind/path (2)", sqlite3_errmsg(db));
  }

  return new SQLiteValueIterator(stmt);
}


sqlite3_stmt *
SQLiteConfiguration::prepare_update_value(const char *sql,
					  const char *path)
{
  sqlite3_stmt *stmt;
  const char   *tail;

  if ( sqlite3_prepare(db, sql, -1, &stmt, &tail) != SQLITE_OK ) {
    throw ConfigurationException("prepare_update_value/prepare", sqlite3_errmsg(db));
  }
  if ( sqlite3_bind_text(stmt, 2, path, -1, NULL) != SQLITE_OK ) {
    ConfigurationException ce("prepare_update_value/bind", sqlite3_errmsg(db));
    sqlite3_finalize(stmt);
    throw ce;
  }

  return stmt;
}


sqlite3_stmt *
SQLiteConfiguration::prepare_insert_value(const char *sql, const char *type,
					  const char *path)
{
  sqlite3_stmt *stmt;
  const char   *tail;

  if ( sqlite3_prepare(db, sql, -1, &stmt, &tail) != SQLITE_OK ) {
    throw ConfigurationException("prepare_insert_value/prepare", sqlite3_errmsg(db));
  }
  if ( (sqlite3_bind_text(stmt, 1, path, -1, NULL) != SQLITE_OK) ||
       (sqlite3_bind_text(stmt, 2, type, -1, NULL) != SQLITE_OK) ) {
    ConfigurationException ce("prepare_insert_value/bind", sqlite3_errmsg(db));
    sqlite3_finalize(stmt);
    throw ce;
  }

  return stmt;
}


void
SQLiteConfiguration::execute_insert_or_update(sqlite3_stmt *stmt)
{
  if ( sqlite3_step(stmt) != SQLITE_DONE ) {
    ConfigurationException ce("execute_insert_or_update", sqlite3_errmsg(db));
    sqlite3_finalize(stmt);
    throw ce;
  }
}


/** Set new value in configuration of type float
 * @param path path to value
 * @param f new value
 */
void
SQLiteConfiguration::set_float(const char *path, float f)
{
  sqlite3_stmt *stmt = NULL;

  mutex->lock();

  try {
    stmt = prepare_update_value(SQL_UPDATE_VALUE, path);
    if ( (sqlite3_bind_double(stmt, 1, f) != SQLITE_OK) ) {
      ConfigurationException ce("set_float/update/bind", sqlite3_errmsg(db));
      sqlite3_finalize(stmt);
      mutex->unlock();
      throw ce;
    }
    execute_insert_or_update(stmt);
    sqlite3_finalize(stmt);
  } catch (Exception &e) {
    if ( stmt != NULL ) sqlite3_finalize(stmt);
    mutex->unlock();
    throw;
  }

  if ( sqlite3_changes(db) == 0 ) {
    // value did not exist, insert

    try {
      stmt = prepare_insert_value(SQL_INSERT_VALUE, "float", path);
      if ( (sqlite3_bind_double(stmt, 3, f) != SQLITE_OK) ) {
	ConfigurationException ce("set_float/insert/bind", sqlite3_errmsg(db));
	sqlite3_finalize(stmt);
	mutex->unlock();
	throw ce;
      }
      execute_insert_or_update(stmt);
      sqlite3_finalize(stmt);
    } catch (Exception &e) {
      if ( stmt != NULL ) sqlite3_finalize(stmt);
      mutex->unlock();
      throw;
    }
  }

  mutex->unlock();

  ChangeHandlerList *h = find_handlers(path);
  for (ChangeHandlerList::const_iterator i = h->begin(); i != h->end(); ++i) {
    (*i)->config_value_changed(path, f);
  }
  delete h;
}


/** Set new value in configuration of type unsigned int
 * @param path path to value
 * @param uint new value
 */
void
SQLiteConfiguration::set_uint(const char *path, unsigned int uint)
{
  sqlite3_stmt *stmt = NULL;

  mutex->lock();

  try {
    stmt = prepare_update_value(SQL_UPDATE_VALUE, path);
    if ( (sqlite3_bind_int(stmt, 1, uint) != SQLITE_OK) ) {
      ConfigurationException ce("set_uint/update/bind", sqlite3_errmsg(db));
      sqlite3_finalize(stmt);
      mutex->unlock();
      throw ce;
    }
    execute_insert_or_update(stmt);
    sqlite3_finalize(stmt);
  } catch (Exception &e) {
    if ( stmt != NULL ) sqlite3_finalize(stmt);
    mutex->unlock();
    throw;
  }

  if ( sqlite3_changes(db) == 0 ) {
    // value did not exist, insert

    try {
      stmt = prepare_insert_value(SQL_INSERT_VALUE, "unsigned int", path);
      if ( (sqlite3_bind_int(stmt, 3, uint) != SQLITE_OK) ) {
	ConfigurationException ce("set_uint/insert/bind", sqlite3_errmsg(db));
	sqlite3_finalize(stmt);
	mutex->unlock();
	throw ce;
      }
      execute_insert_or_update(stmt);
      sqlite3_finalize(stmt);
    } catch (Exception &e) {
      if ( stmt != NULL ) sqlite3_finalize(stmt);
      mutex->unlock();
      throw;
    }
  }
  mutex->unlock();

  ChangeHandlerList *h = find_handlers(path);
  for (ChangeHandlerList::const_iterator i = h->begin(); i != h->end(); ++i) {
    (*i)->config_value_changed(path, uint);
  }
  delete h;
}


/** Set new value in configuration of type int
 * @param path path to value
 * @param i new value
 */
void
SQLiteConfiguration::set_int(const char *path, int i)
{
  sqlite3_stmt *stmt = NULL;

  mutex->lock();

  try {
    stmt = prepare_update_value(SQL_UPDATE_VALUE, path);
    if ( (sqlite3_bind_int(stmt, 1, i) != SQLITE_OK) ) {
      ConfigurationException ce("set_int/update/bind", sqlite3_errmsg(db));
      sqlite3_finalize(stmt);
      mutex->unlock();
      throw ce;
    }
    execute_insert_or_update(stmt);
    sqlite3_finalize(stmt);
  } catch (Exception &e) {
    if ( stmt != NULL ) sqlite3_finalize(stmt);
    mutex->unlock();
    throw;
  }

  if ( sqlite3_changes(db) == 0 ) {
    // value did not exist, insert

    try {
      stmt = prepare_insert_value(SQL_INSERT_VALUE, "int", path);
      if ( (sqlite3_bind_int(stmt, 3, i) != SQLITE_OK) ) {
	ConfigurationException ce("set_int/insert/bind", sqlite3_errmsg(db));
	sqlite3_finalize(stmt);
	mutex->unlock();
	throw ce;
      }
      execute_insert_or_update(stmt);
      sqlite3_finalize(stmt);
    } catch (Exception &e) {
      if ( stmt != NULL ) sqlite3_finalize(stmt);
      mutex->unlock();
      throw;
    }
  }

  mutex->unlock();

  ChangeHandlerList *h = find_handlers(path);
  for (ChangeHandlerList::const_iterator j = h->begin(); j != h->end(); ++j) {
    (*j)->config_value_changed(path, i);
  }
  delete h;
}


/** Set new value in configuration of type bool
 * @param path path to value
 * @param b new value
 */
void
SQLiteConfiguration::set_bool(const char *path, bool b)
{
  sqlite3_stmt *stmt = NULL;

  mutex->lock();

  try {
    stmt = prepare_update_value(SQL_UPDATE_VALUE, path);
    if ( (sqlite3_bind_int(stmt, 1, (b ? 1 : 0)) != SQLITE_OK) ) {
      ConfigurationException ce("set_bool/update/bind", sqlite3_errmsg(db));
      sqlite3_finalize(stmt);
      mutex->unlock();
      throw ce;
    }
    execute_insert_or_update(stmt);
    sqlite3_finalize(stmt);
  } catch (Exception &e) {
    if ( stmt != NULL ) sqlite3_finalize(stmt);
    mutex->unlock();
    throw;
  }

  if ( sqlite3_changes(db) == 0 ) {
    // value did not exist, insert

    try {
      stmt = prepare_insert_value(SQL_INSERT_VALUE, "bool", path);
      if ( (sqlite3_bind_int(stmt, 3, (b ? 1 : 0)) != SQLITE_OK) ) {
	ConfigurationException ce("set_bool/insert/bind", sqlite3_errmsg(db));
	sqlite3_finalize(stmt);
	mutex->unlock();
	throw ce;
      }
      execute_insert_or_update(stmt);
      sqlite3_finalize(stmt);
    } catch (Exception &e) {
      if ( stmt != NULL ) sqlite3_finalize(stmt);
      mutex->unlock();
      throw;
    }
  }

  mutex->unlock();

  ChangeHandlerList *h = find_handlers(path);
  for (ChangeHandlerList::const_iterator i = h->begin(); i != h->end(); ++i) {
    (*i)->config_value_changed(path, b);
  }
  delete h;
}


void
SQLiteConfiguration::set_string(const char *path,
				const char *s)
{
  sqlite3_stmt *stmt = NULL;

  mutex->lock();

  size_t s_length = strlen(s);

  try {
    stmt = prepare_update_value(SQL_UPDATE_VALUE, path);
    if ( (sqlite3_bind_text(stmt, 1, s, s_length, SQLITE_STATIC) != SQLITE_OK) ) {
      ConfigurationException ce("set_string/update/bind", sqlite3_errmsg(db));
      sqlite3_finalize(stmt);
      mutex->unlock();
      throw ce;
    }
    execute_insert_or_update(stmt);
    sqlite3_finalize(stmt);
  } catch (Exception &e) {
    if ( stmt != NULL ) sqlite3_finalize(stmt);
    mutex->unlock();
    throw;
  }

  if ( sqlite3_changes(db) == 0 ) {
    // value did not exist, insert

    try {
      stmt = prepare_insert_value(SQL_INSERT_VALUE, "string", path);
      if ( (sqlite3_bind_text(stmt, 3, s, s_length, SQLITE_STATIC) != SQLITE_OK) ) {
	ConfigurationException ce("set_string/insert/bind", sqlite3_errmsg(db));
	sqlite3_finalize(stmt);
	mutex->unlock();
	throw ce;
      }
      execute_insert_or_update(stmt);
      sqlite3_finalize(stmt);
    } catch (Exception &e) {
      if ( stmt != NULL ) sqlite3_finalize(stmt);
      mutex->unlock();
      throw;
    }
  }

  mutex->unlock();

  ChangeHandlerList *h = find_handlers(path);
  for (ChangeHandlerList::const_iterator i = h->begin(); i != h->end(); ++i) {
    (*i)->config_value_changed(path, s);
  }
  delete h;
}


/** Set new value in configuration of type string
 * @param path path to value
 * @param s new value
 */
void
SQLiteConfiguration::set_string(const char *path, std::string s)
{
  set_string(path, s.c_str());
}


/** Erase the given value from the configuration. It is not an error if the value does
 * not exists before deletion.
 * @param path path to value
 */
void
SQLiteConfiguration::erase(const char *path)
{
  sqlite3_stmt *stmt;
  const char   *tail;

  if ( sqlite3_prepare(db, SQL_DELETE_VALUE, -1, &stmt, &tail) != SQLITE_OK ) {
    throw ConfigurationException("erase/prepare", sqlite3_errmsg(db));
  }
  if ( sqlite3_bind_text(stmt, 1, path, -1, NULL) != SQLITE_OK ) {
    ConfigurationException ce("erase/bind", sqlite3_errmsg(db));
    sqlite3_finalize(stmt);
    throw ce;
  }

  if ( sqlite3_step(stmt) != SQLITE_DONE ) {
    ConfigurationException ce("erase/execute", sqlite3_errmsg(db));
    sqlite3_finalize(stmt);
    throw ce;    
  }

  sqlite3_finalize(stmt);

  ChangeHandlerList *h = find_handlers(path);
  for (ChangeHandlerList::const_iterator i = h->begin(); i != h->end(); ++i) {
    (*i)->config_value_erased(path);
  }
  delete h;
}


void
SQLiteConfiguration::set_default_float(const char *path, float f)
{
  sqlite3_stmt *stmt = NULL;

  mutex->lock();

  try {
    stmt = prepare_update_value(SQL_UPDATE_DEFAULT_VALUE, path);
    if ( (sqlite3_bind_double(stmt, 1, f) != SQLITE_OK) ) {
      ConfigurationException ce("set_default_float/update/bind", sqlite3_errmsg(db));
      sqlite3_finalize(stmt);
      mutex->unlock();
      throw ce;
    }
    execute_insert_or_update(stmt);
    sqlite3_finalize(stmt);
  } catch (Exception &e) {
    if ( stmt != NULL ) sqlite3_finalize(stmt);
    mutex->unlock();
    throw;
  }

  if ( sqlite3_changes(db) == 0 ) {
    // value did not exist, insert

    try {
      stmt = prepare_insert_value(SQL_INSERT_DEFAULT_VALUE, "float", path);
      if ( (sqlite3_bind_double(stmt, 3, f) != SQLITE_OK) ) {
	ConfigurationException ce("set_default_float/insert/bind", sqlite3_errmsg(db));
	sqlite3_finalize(stmt);
	mutex->unlock();
	throw ce;
      }
      execute_insert_or_update(stmt);
      sqlite3_finalize(stmt);
    } catch (Exception &e) {
      if ( stmt != NULL ) sqlite3_finalize(stmt);
      mutex->unlock();
      throw;
    }
  }

  mutex->unlock();

  ChangeHandlerList *h = find_handlers(path);
  for (ChangeHandlerList::const_iterator i = h->begin(); i != h->end(); ++i) {
    (*i)->config_value_changed(path, f);
  }
  delete h;
}


void
SQLiteConfiguration::set_default_uint(const char *path, unsigned int uint)
{
  sqlite3_stmt *stmt = NULL;

  mutex->lock();

  try {
    stmt = prepare_update_value(SQL_UPDATE_DEFAULT_VALUE, path);
    if ( (sqlite3_bind_int(stmt, 1, uint) != SQLITE_OK) ) {
      ConfigurationException ce("set_default_uint/update/bind", sqlite3_errmsg(db));
      sqlite3_finalize(stmt);
      mutex->unlock();
      throw ce;
    }
    execute_insert_or_update(stmt);
    sqlite3_finalize(stmt);
  } catch (Exception &e) {
    if ( stmt != NULL ) sqlite3_finalize(stmt);
    mutex->unlock();
    throw;
  }

  if ( sqlite3_changes(db) == 0 ) {
    // value did not exist, insert

    try {
      stmt = prepare_insert_value(SQL_INSERT_DEFAULT_VALUE, "unsigned int", path);
      if ( (sqlite3_bind_int(stmt, 3, uint) != SQLITE_OK) ) {
	ConfigurationException ce("set_default_uint/insert/bind", sqlite3_errmsg(db));
	sqlite3_finalize(stmt);
	mutex->unlock();
	throw ce;
      }
      execute_insert_or_update(stmt);
      sqlite3_finalize(stmt);
    } catch (Exception &e) {
      if ( stmt != NULL ) sqlite3_finalize(stmt);
      mutex->unlock();
      throw;
    }
  }
  mutex->unlock();

  ChangeHandlerList *h = find_handlers(path);
  for (ChangeHandlerList::const_iterator i = h->begin(); i != h->end(); ++i) {
    (*i)->config_value_changed(path, uint);
  }
  delete h;
}


void
SQLiteConfiguration::set_default_int(const char *path, int i)
{
  sqlite3_stmt *stmt = NULL;
  mutex->lock();

  try {
    stmt = prepare_update_value(SQL_UPDATE_DEFAULT_VALUE, path);
    if ( (sqlite3_bind_int(stmt, 1, i) != SQLITE_OK) ) {
      ConfigurationException ce("set_default_int/update/bind", sqlite3_errmsg(db));
      sqlite3_finalize(stmt);
      mutex->unlock();
      throw ce;
    }
    execute_insert_or_update(stmt);
    sqlite3_finalize(stmt);
  } catch (Exception &e) {
    if ( stmt != NULL ) sqlite3_finalize(stmt);
    mutex->unlock();
    throw;
  }

  if ( sqlite3_changes(db) == 0 ) {
    // value did not exist, insert
    try {
      stmt = prepare_insert_value(SQL_INSERT_DEFAULT_VALUE, "int", path);
      if ( (sqlite3_bind_int(stmt, 3, i) != SQLITE_OK) ) {
	ConfigurationException ce("set_default_int/insert/bind", sqlite3_errmsg(db));
	sqlite3_finalize(stmt);
	mutex->unlock();
	throw ce;
      }
      execute_insert_or_update(stmt);
      sqlite3_finalize(stmt);
    } catch (Exception &e) {
      if ( stmt != NULL ) sqlite3_finalize(stmt);
      mutex->unlock();
      throw;
    }
  }

  mutex->unlock();

  ChangeHandlerList *h = find_handlers(path);
  for (ChangeHandlerList::const_iterator j = h->begin(); j != h->end(); ++j) {
    (*j)->config_value_changed(path, i);
  }
  delete h;
}


void
SQLiteConfiguration::set_default_bool(const char *path, bool b)
{
  sqlite3_stmt *stmt = NULL;

  mutex->lock();

  try {
    stmt = prepare_update_value(SQL_UPDATE_DEFAULT_VALUE, path);
    if ( (sqlite3_bind_int(stmt, 1, (b ? 1 : 0)) != SQLITE_OK) ) {
      ConfigurationException ce("set_default_bool/update/bind", sqlite3_errmsg(db));
      sqlite3_finalize(stmt);
      mutex->unlock();
      throw ce;
    }
    execute_insert_or_update(stmt);
    sqlite3_finalize(stmt);
  } catch (Exception &e) {
    if ( stmt != NULL ) sqlite3_finalize(stmt);
    mutex->unlock();
    throw;
  }

  if ( sqlite3_changes(db) == 0 ) {
    // value did not exist, insert

    try {
      stmt = prepare_insert_value(SQL_INSERT_DEFAULT_VALUE, "bool", path);
      if ( (sqlite3_bind_int(stmt, 3, (b ? 1 : 0)) != SQLITE_OK) ) {
	ConfigurationException ce("set_default_bool/insert/bind", sqlite3_errmsg(db));
	sqlite3_finalize(stmt);
	mutex->unlock();
	throw ce;
      }
      execute_insert_or_update(stmt);
      sqlite3_finalize(stmt);
    } catch (Exception &e) {
      if ( stmt != NULL ) sqlite3_finalize(stmt);
      mutex->unlock();
      throw;
    }
  }

  mutex->unlock();

  ChangeHandlerList *h = find_handlers(path);
  for (ChangeHandlerList::const_iterator i = h->begin(); i != h->end(); ++i) {
    (*i)->config_value_changed(path, b);
  }
  delete h;
}


void
SQLiteConfiguration::set_default_string(const char *path,
					const char *s)
{
  sqlite3_stmt *stmt = NULL;

  mutex->lock();
  size_t s_length = strlen(s);

  try {
    stmt = prepare_update_value(SQL_UPDATE_DEFAULT_VALUE, path);
    if ( (sqlite3_bind_text(stmt, 1, s, s_length, SQLITE_STATIC) != SQLITE_OK) ) {
      ConfigurationException ce("set_default_string/update/bind", sqlite3_errmsg(db));
      sqlite3_finalize(stmt);
      mutex->unlock();
      throw ce;
    }
    execute_insert_or_update(stmt);
    sqlite3_finalize(stmt);
  } catch (Exception &e) {
    if ( stmt != NULL ) sqlite3_finalize(stmt);
    mutex->unlock();
    throw;
  }

  if ( sqlite3_changes(db) == 0 ) {
    // value did not exist, insert

    try {
      stmt = prepare_insert_value(SQL_INSERT_DEFAULT_VALUE, "string", path);
      if ( (sqlite3_bind_text(stmt, 3, s, s_length, SQLITE_STATIC) != SQLITE_OK) ) {
	ConfigurationException ce("set_default_string/insert/bind", sqlite3_errmsg(db));
	sqlite3_finalize(stmt);
	mutex->unlock();
	throw ce;
      }
      execute_insert_or_update(stmt);
      sqlite3_finalize(stmt);
    } catch (Exception &e) {
      if ( stmt != NULL ) sqlite3_finalize(stmt);
      mutex->unlock();
      throw;
    }
  }

  mutex->unlock();

  ChangeHandlerList *h = find_handlers(path);
  for (ChangeHandlerList::const_iterator i = h->begin(); i != h->end(); ++i) {
    (*i)->config_value_changed(path, s);
  }
  delete h;
}


void
SQLiteConfiguration::set_default_string(const char *path, std::string s)
{
  set_string(path, s.c_str());
}


void
SQLiteConfiguration::erase_default(const char *path)
{
  sqlite3_stmt *stmt;
  const char   *tail;

  if ( sqlite3_prepare(db, SQL_DELETE_DEFAULT_VALUE, -1, &stmt, &tail) != SQLITE_OK ) {
    throw ConfigurationException("erase_default/prepare", sqlite3_errmsg(db));
  }
  if ( sqlite3_bind_text(stmt, 1, path, -1, NULL) != SQLITE_OK ) {
    ConfigurationException ce("erase_default/bind", sqlite3_errmsg(db));
    sqlite3_finalize(stmt);
    throw ce;
  }

  if ( sqlite3_step(stmt) != SQLITE_DONE ) {
    ConfigurationException ce("erase_default/execute", sqlite3_errmsg(db));
    sqlite3_finalize(stmt);
    throw ce;    
  }

  sqlite3_finalize(stmt);

  ChangeHandlerList *h = find_handlers(path);
  for (ChangeHandlerList::const_iterator i = h->begin(); i != h->end(); ++i) {
    (*i)->config_value_erased(path);
  }
  delete h;
}


/** Lock the config.
 * No further changes or queries can be executed on the configuration and will block until
 * the config is unlocked.
 */
void
SQLiteConfiguration::lock()
{
  mutex->lock();
}


/** Try to lock the config.
 * @see Configuration::lock()
 * @return true, if the lock has been aquired, false otherwise
 */
bool
SQLiteConfiguration::try_lock()
{
  return mutex->try_lock();
}

/** Unlock the config.
 * Modifications and queries are possible again.
 */
void
SQLiteConfiguration::unlock()
{
  mutex->unlock();
}


/** Iterator for all values.
 * Returns an iterator that can be used to iterate over all values in the current
 * configuration.
 * @return iterator over all values
 */
Configuration::ValueIterator *
SQLiteConfiguration::iterator()
{
  sqlite3_stmt *stmt;
  const char *tail;

  if ( sqlite3_prepare(db, SQL_SELECT_ALL, -1, &stmt, &tail) != SQLITE_OK ) {
    throw ConfigurationException("begin: Preparation SQL failed");
  }

  return new SQLiteValueIterator(stmt);
}


/** Iterator with search results.
 * Returns an iterator that can be used to iterate over the search results. All values
 * whose component and path start with the given strings are returned.
 * A call like
 * @code
 *   config->search("");
 * @endcode
 * is effectively the same as a call to iterator().
 * @param path start of path
 * @return iterator to search results
 */
Configuration::ValueIterator *
SQLiteConfiguration::search(const char *path)
{
  sqlite3_stmt *stmt;
  const char *tail;

  char *p;
  if ( asprintf(&p, "%s%%", path) == -1 ) {
    throw ConfigurationException("search: could not allocate component string");
  }

  if ( sqlite3_prepare(db, SQL_SELECT_COMPLETE, -1, &stmt, &tail) != SQLITE_OK ) {
    free(p);
    throw ConfigurationException("begin: Preparation SQL failed");
  }
  if ( sqlite3_bind_text(stmt, 1, p, -1, NULL) != SQLITE_OK ) {
    free(p);
    throw ConfigurationException("begin: Binding text for path failed (1)");
  }
  if ( sqlite3_bind_text(stmt, 2, p, -1, NULL) != SQLITE_OK ) {
    free(p);
    throw ConfigurationException("begin: Binding text for path failed (2)");
  }

  free(p);

  return new SQLiteValueIterator(stmt);
}

/** @class SQLiteConfiguration::SQLiteValueIterator config/sqlite.h
 * SQLite configuration value iterator.
 */


/** Constructor.
 * @param stmt compiled SQLite statement
 */
SQLiteConfiguration::SQLiteValueIterator::SQLiteValueIterator(sqlite3_stmt *stmt)
{
  this->stmt = stmt;
}


/** Destructor. */
SQLiteConfiguration::SQLiteValueIterator::~SQLiteValueIterator()
{
  if ( stmt != NULL ) {
    sqlite3_finalize(stmt);
    stmt = NULL;
  }
}


/* Check if there is another element and advance to this if possible.
 * This advances to the next element, if there is one.
 * @return true, if another element has been reached, false otherwise
 */
bool
SQLiteConfiguration::SQLiteValueIterator::next()
{
  if ( stmt == NULL)  return false;

  if ( sqlite3_step(stmt) == SQLITE_ROW ) {
    return true;
  } else {
    sqlite3_finalize(stmt);
    stmt = NULL;
    return false;
  }
}

/** Check if the current element is valid.
 * This is much like the classic end element for iterators. If the iterator is
 * invalid there all subsequent calls to next() shall fail.
 * @return true, if the iterator is still valid, false otherwise
 */
bool
SQLiteConfiguration::SQLiteValueIterator::valid()
{
  return ( stmt != NULL);
}


/** Path of value.
 * @return path of value
 */
const char *
SQLiteConfiguration::SQLiteValueIterator::path()
{
  return (const char *)sqlite3_column_text(stmt, 0);
}


/** Type of value.
 * @return string representation of value type.
 */
const char *
SQLiteConfiguration::SQLiteValueIterator::type()
{
  return (const char *)sqlite3_column_text(stmt, 1);
}


/** Check if current value is a float.
 * @return true, if value is a float, false otherwise
 */
bool
SQLiteConfiguration::SQLiteValueIterator::is_float()
{
  return (strcmp("float", (const char *)sqlite3_column_text(stmt, 1)) == 0);
}


/** Check if current value is a unsigned int.
 * @return true, if value is a unsigned int, false otherwise
 */
bool
SQLiteConfiguration::SQLiteValueIterator::is_uint()
{
  return (strcmp("unsigned int", (const char *)sqlite3_column_text(stmt, 1)) == 0);
}

/** Check if current value is a int.
 * @return true, if value is a int, false otherwise
 */
bool
SQLiteConfiguration::SQLiteValueIterator::is_int()
{
  return (strcmp("int", (const char *)sqlite3_column_text(stmt, 1)) == 0);
}


/** Check if current value is a bool.
 * @return true, if value is a bool, false otherwise
 */
bool
SQLiteConfiguration::SQLiteValueIterator::is_bool()
{
  return (strcmp("bool", (const char *)sqlite3_column_text(stmt, 1)) == 0);
}


/** Check if current value is a string.
 * @return true, if value is a string, false otherwise
 */
bool
SQLiteConfiguration::SQLiteValueIterator::is_string()
{
  return (strcmp("string", (const char *)sqlite3_column_text(stmt, 1)) == 0);
}


/** Get float value.
 * @return value
 */
float
SQLiteConfiguration::SQLiteValueIterator::get_float()
{
  return (float)sqlite3_column_double(stmt, 2);
}


/** Get unsigned int value.
 * @return value
 */
unsigned int
SQLiteConfiguration::SQLiteValueIterator::get_uint()
{
  int i = sqlite3_column_int(stmt, 2);
  if( i < 0 ) {
    return 0;
  } else {
    return i;
  }
}


/** Get int value.
 * @return value
 */
int
SQLiteConfiguration::SQLiteValueIterator::get_int()
{
  return sqlite3_column_int(stmt, 2);
}

/** Get bool value.
 * @return value
 */
bool
SQLiteConfiguration::SQLiteValueIterator::get_bool()
{
  return (sqlite3_column_int(stmt, 2) != 0);
}

/** Get string value.
 * @return value
 */
std::string
SQLiteConfiguration::SQLiteValueIterator::get_string()
{
  return (const char *)sqlite3_column_text(stmt, 2);
}


