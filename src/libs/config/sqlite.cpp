
/***************************************************************************
 *  sqlite.cpp - Fawkes configuration stored in a SQLite database
 *
 *  Created: Wed Dec 06 17:23:00 2006
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

#include <config/sqlite.h>
#include <core/threading/mutex.h>
#include <core/exceptions/system.h>
#include <core/exceptions/software.h>

#include <sqlite3.h>

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <fnmatch.h>

namespace fawkes {

/* SQLite statements */

#define TABLE_HOST_CONFIG "config"
#define TABLE_DEFAULT_CONFIG "defaults.config"

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

#define SQL_CREATE_TABLE_MODIFIED_CONFIG				\
  "CREATE TABLE IF NOT EXISTS modified.config (\n"			\
  "  path      TEXT NOT NULL,\n"					\
  "  type      TEXT NOT NULL,\n"					\
  "  value     NOT NULL,\n"						\
  "  comment   TEXT,\n"							\
  "  modtype   TEXT NOT NULL,\n"					\
  "  oldvalue  NOT NULL,\n"						\
  "  PRIMARY KEY (path)\n"						\
  ")"

#define SQL_ATTACH_DEFAULTS						\
  "ATTACH DATABASE '%s' AS defaults"

#define SQL_ATTACH_MODIFIED						\
  "ATTACH DATABASE ':memory:' AS modified"

#define SQL_ATTACH_DUMPED						\
  "ATTACH DATABASE '%s' AS dumped"

#define SQL_DETACH_DUMPED						\
  "DETACH DATABASE dumped"

#define SQL_SELECT_VALUE_TYPE						\
  "SELECT type, value, 0 AS is_default FROM config WHERE path=? UNION "	\
  "SELECT type, value, 1 AS is_default FROM defaults.config AS dc "	\
  "WHERE path=? AND NOT EXISTS "					\
  "(SELECT path FROM config WHERE dc.path=path)"

#define SQL_SELECT_COMPLETE						\
  "SELECT *, 0 AS is_default FROM config WHERE path LIKE ? UNION "	\
  "SELECT *, 1 AS is_default FROM defaults.config AS dc "		\
  "WHERE path LIKE ? AND NOT EXISTS "					\
  "(SELECT path FROM config WHERE dc.path = path) "			\
  "ORDER BY path"

#define SQL_SELECT_TYPE							\
  "SELECT type, 0 AS is_default FROM config WHERE path=? UNION "	\
  "SELECT type, 1 AS is_default FROM defaults.config AS dc "		\
  "WHERE path=? AND NOT EXISTS "					\
  "(SELECT path FROM config WHERE dc.path = path)"

#define SQL_SELECT_COMMENT						\
  "SELECT comment, 0 AS is_default FROM config WHERE path=?"

#define SQL_SELECT_DEFAULT_COMMENT					\
  "SELECT comment, 1 AS is_default FROM defaults.config AS dc "		\
  "WHERE dc.path=?"

#define SQL_UPDATE_VALUE						\
  "UPDATE config SET value=? WHERE path=?"

#define SQL_UPDATE_DEFAULT_VALUE					\
  "UPDATE defaults.config SET value=? WHERE path=?"

#define SQL_UPDATE_COMMENT						\
  "UPDATE config SET comment=? WHERE path=?"

#define SQL_UPDATE_DEFAULT_COMMENT					\
  "UPDATE defaults.config SET comment=? WHERE path=?"

#define SQL_INSERT_VALUE						\
  "INSERT INTO config (path, type, value) VALUES (?, ?, ?)"

#define SQL_INSERT_DEFAULT_VALUE					\
  "INSERT INTO defaults.config (path, type, value) VALUES (?, ?, ?)"

#define SQL_SELECT_ALL							\
  "SELECT *, 0 AS is_default FROM config UNION "			\
  "SELECT *, 1 AS is_default FROM defaults.config AS dc "		\
  "WHERE NOT EXISTS "							\
  "(SELECT path FROM config WHERE dc.path = path) "			\
  "ORDER BY path"

#define SQL_SELECT_ALL_DEFAULT						\
  "SELECT *, 1 AS is_default FROM defaults.config"

#define SQL_SELECT_ALL_HOSTSPECIFIC					\
  "SELECT *, 0 AS is_default FROM config"

#define SQL_DELETE_VALUE						\
  "DELETE FROM config WHERE path=?"

#define SQL_DELETE_DEFAULT_VALUE					\
  "DELETE FROM defaults.config WHERE path=?"

#define SQL_UPDATE_DEFAULT_DB						\
  "INSERT INTO config SELECT * FROM defaults.config AS dc "		\
  "WHERE NOT EXISTS (SELECT path from config WHERE path = dc.path)"

#define SQL_UPDATE_MODIFIED_DB_ADDED					\
  "INSERT INTO modified.config "					\
  "  SELECT duc.*,'added' AS modtype, duc.value "			\
  "    FROM dumped.config AS duc "					\
  "    WHERE NOT EXISTS (SELECT dc.path FROM defaults.config AS dc "	\
  "                        WHERE dc.path=duc.path) "			\
  "    ORDER BY path"

#define SQL_UPDATE_MODIFIED_DB_ERASED					\
  "INSERT INTO modified.config "					\
  "  SELECT dc.*,'erased' AS modtype, dc.value "			\
  "    FROM defaults.config AS dc "					\
  "    WHERE NOT EXISTS (SELECT duc.path FROM dumped.config AS duc "	\
  "                        WHERE duc.path=dc.path) "			\
  "    ORDER BY path"

#define SQL_UPDATE_MODIFIED_DB_CHANGED					\
  "INSERT INTO modified.config "					\
  "  SELECT duc.*,'changed' AS modtype, dc.value "			\
  "    FROM dumped.config AS duc, defaults.config AS dc "		\
  "    WHERE duc.path = dc.path "					\
  "      AND (dc.type != duc.type OR dc.value != duc.value) "		\
  "    ORDER BY duc.path"

#define SQL_COPY_DUMP							\
  "DELETE FROM defaults.config; "					\
  "INSERT INTO defaults.config SELECT * FROM dumped.config"

#define SQL_SELECT_MODIFIED_ALL						\
  "SELECT * FROM modified.config"

/** @class SQLiteConfiguration <config/sqlite.h>
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


/** Constructor. */
SQLiteConfiguration::SQLiteConfiguration()
{
  opened = false;
  mutex = new Mutex();

  __sysconfdir   = NULL;
  __userconfdir  = NULL;
  __default_file = NULL;
  __default_sql  = NULL;

}

/** Constructor.
 * @param sysconfdir system configuration directory, will be searched for
 * default configuration file, and system will try to create host-specific
 * database if writable
 * @param userconfdir user configuration directory, will be searched preferably
 * for default configuration file, and will be used to create host-specific
 * database if sysconfdir is not writable. This directory will be created
 * if it does not exist during load().
 */
SQLiteConfiguration::SQLiteConfiguration(const char *sysconfdir,
					 const char *userconfdir)
{
  opened = false;
  mutex = new Mutex();

  __sysconfdir   = strdup(sysconfdir);
  __default_file = NULL;
  __default_sql  = NULL;

  if (userconfdir != NULL) {
    __userconfdir  = strdup(userconfdir);
  } else {
    const char *homedir = getenv("HOME");
    if (homedir == NULL) {
      __userconfdir = strdup(sysconfdir);
    } else {
      if (asprintf(&__userconfdir, "%s/%s", homedir, USERDIR) == -1) {
	__userconfdir = strdup(sysconfdir);
      }
    }
  }
}

/** Destructor. */
SQLiteConfiguration::~SQLiteConfiguration()
{
  if (opened) {
    opened = false;
    if ( sqlite3_close(db) == SQLITE_BUSY ) {
      printf("Boom, we are dead, database cannot be closed "
	     "because there are open handles\n");
    }
  }

  if (__host_file)    free(__host_file);
  if (__default_file) free(__default_file);
  if (__default_sql)  free(__default_sql);
  if (__sysconfdir)   free(__sysconfdir);
  if (__userconfdir)  free(__userconfdir);
  delete mutex;
}


/** Initialize the configuration database(s).
 * Initialize databases. If the host-specific database already exists
 * an exception is thrown. You have to delete it before calling
 * init().  First the host-specific database is created. It will
 * contain one table, named 'config'. The 'config' table will hold the
 * current configuration for this machine.
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
 * If no default database exists it is created. The database is kept in a file
 * called default.db. It contains a single table called 'config' with the same
 * structure as the 'config' table in the host-specific database.
 */
void
SQLiteConfiguration::init_dbs()
{
  char *errmsg;
  if ( (sqlite3_exec(db, SQL_CREATE_TABLE_HOST_CONFIG, NULL, NULL, &errmsg) != SQLITE_OK) ||
       (sqlite3_exec(db, SQL_CREATE_TABLE_DEFAULT_CONFIG, NULL, NULL, &errmsg) != SQLITE_OK) ) {
    CouldNotOpenConfigException ce(sqlite3_errmsg(db));
    sqlite3_close(db);
    throw ce;
  }
}


/** Dump table.
 * Dumps a table to the given file.
 * @param f file to write to
 * @param tdb SQLite3 database to read from
 * @param table_name Name of the table to dump
 */
static void
dump_table(FILE *f, ::sqlite3 *tdb, const char *table_name)
{
  std::string tisql = "PRAGMA table_info(\"";
  tisql += table_name;
  tisql += "\");";

  sqlite3_stmt *stmt;
  if ( sqlite3_prepare(tdb, tisql.c_str(), -1, &stmt, 0) != SQLITE_OK ) {
    throw ConfigurationException("dump_table/prepare", sqlite3_errmsg(tdb));
  }
  std::string value_query = "SELECT 'INSERT INTO ' || '\"";
  value_query += table_name;
  value_query += "\"' || ' VALUES(' || ";
  int rv = sqlite3_step(stmt);
  while ( rv == SQLITE_ROW ) {
    value_query += "quote(\"";
    value_query += (const char *)sqlite3_column_text(stmt, 1);
    value_query += "\") || ";
    rv = sqlite3_step(stmt);
    if ( rv == SQLITE_ROW ) {
      value_query += " ',' || ";
    }
  }
  value_query += "')' FROM ";
  value_query += table_name;
  sqlite3_finalize(stmt);

  sqlite3_stmt *vstmt;
  if ( sqlite3_prepare(tdb, value_query.c_str(), -1, &vstmt, 0) != SQLITE_OK ) {
    throw ConfigurationException("dump_table/prepare 2", sqlite3_errmsg(tdb));
  }
  while ( sqlite3_step(vstmt) == SQLITE_ROW ) {
    fprintf(f, "%s;\n", sqlite3_column_text(vstmt, 0));
  }
  sqlite3_finalize(vstmt);
}

void
SQLiteConfiguration::dump(::sqlite3 *tdb, const char *dumpfile)
{
  FILE *f = fopen(dumpfile, "w");
  if ( ! f ) {
    throw CouldNotOpenFileException(dumpfile, errno, "Could not open dump file");
  }

  fprintf(f, "BEGIN TRANSACTION;\n");

  const char *sql = "SELECT name, sql FROM sqlite_master "
                    "WHERE sql NOT NULL AND type=='table'";
  sqlite3_stmt *stmt;
  if ( (sqlite3_prepare(tdb, sql, -1, &stmt, 0) != SQLITE_OK) || ! stmt ) {
    throw ConfigurationException("dump_query/prepare", sqlite3_errmsg(tdb));
  }
  while ( sqlite3_step(stmt) == SQLITE_ROW ) {
    fprintf(f, "%s;\n", sqlite3_column_text(stmt, 1));
    dump_table(f, tdb, (const char *)sqlite3_column_text(stmt, 0));
  }
  sqlite3_finalize(stmt);

  fprintf(f, "COMMIT;\n");
  fclose(f);
}


/** Try to dump default configuration.
 * This method will try to open the SQL dump file for writing and dump
 * the current content of the default database into the file.
 * @exception Exception thrown if dumping fails
 */
void
SQLiteConfiguration::try_dump()
{
  if ( __default_sql ) {
    sqlite3 *tdb;
    if ( sqlite3_open(__default_file, &tdb) == SQLITE_OK ) {
      try {
	dump(tdb, __default_sql);
	sqlite3_close(tdb);
      } catch (Exception &e) {
	sqlite3_close(tdb);
	throw;
      }
    }
  }
}

void
SQLiteConfiguration::import(::sqlite3 *tdb, const char *dumpfile)
{
  FILE *f = fopen(dumpfile, "r");

  if (! f) {
    throw CouldNotOpenConfigException("Import failed, could not open dump file");
  }

  char line[4096];
  char *errmsg;
  while (! feof(f) ) {
    line[0] = 0;
    unsigned int i = 0;
    while (! feof(f) && (i < sizeof(line) - 1)) {
      if (fread(&(line[i]), 1, 1, f) == 1) {
	++i;
	if ( (i > 2) && (line[i-1] == '\n') && (line[i-2] == ';') ) {
	  break;
	}
      } else {
	break;
      }
    }
    line[i] = 0;
    if ( line[0] != 0 ) {
      if ( sqlite3_exec(tdb, line, 0, 0, &errmsg) != SQLITE_OK ) {
	ConfigurationException e(errmsg, line);
	sqlite3_free(errmsg);
	throw e;
      }
    }
  }

  fclose(f);
}


void
SQLiteConfiguration::import_default(const char *default_sql)
{
  char *tmpfile = strdup(TMPDIR"/tmp_default_XXXXXX");
  tmpfile = mktemp(tmpfile);
  if ( tmpfile[0] == 0 ) {
    throw CouldNotOpenConfigException("Failed to create temp file for default DB import");
  }

  // Import .sql file into dump database (temporary file)
  sqlite3 *dump_db;
  if ( sqlite3_open(tmpfile, &dump_db) == SQLITE_OK ) {
    import(dump_db, default_sql);
    sqlite3_close(dump_db);
  } else {
    throw CouldNotOpenConfigException("Failed to import dump file into temp DB");
  }

  // Attach dump database as "dumped"
  char *attach_sql;
  char *errmsg;
  if ( asprintf(&attach_sql, SQL_ATTACH_DUMPED, tmpfile) == -1 ) {
    throw CouldNotOpenConfigException("Could not create attachment SQL in merge");
  }
  if ( sqlite3_exec(db, attach_sql, NULL, NULL, &errmsg) != SQLITE_OK ) {
    free(attach_sql);
    CouldNotOpenConfigException e("Could not attach dump DB in merge: %s", errmsg);
    sqlite3_free(errmsg);
    throw e;
  }
  free(attach_sql);

  // Create "modified" database for a list of modified values, only stored in RAM
  if ( (sqlite3_exec(db, SQL_ATTACH_MODIFIED, NULL, NULL, &errmsg) != SQLITE_OK) ||
       (sqlite3_exec(db, SQL_CREATE_TABLE_MODIFIED_CONFIG, NULL, NULL, &errmsg) != SQLITE_OK) ) {
    CouldNotOpenConfigException ce("Could not create or attach modified memory database: %s", errmsg);
    sqlite3_free(errmsg);
    throw ce;
  }

  // Compare old and new database, copying modifications to "modified" database
  if ( (sqlite3_exec(db, SQL_UPDATE_MODIFIED_DB_ADDED, NULL, NULL, &errmsg) != SQLITE_OK) ||
       (sqlite3_exec(db, SQL_UPDATE_MODIFIED_DB_ERASED, NULL, NULL, &errmsg) != SQLITE_OK) ||
       (sqlite3_exec(db, SQL_UPDATE_MODIFIED_DB_CHANGED, NULL, NULL, &errmsg) != SQLITE_OK) ) {
    CouldNotOpenConfigException ce("Could not update modified memory database: %s", errmsg);
    sqlite3_free(errmsg);
    throw ce;
  }

  // Copy dump to defaults DB, overwriting everything
  if ( (sqlite3_exec(db, SQL_COPY_DUMP, NULL, NULL, &errmsg) != SQLITE_OK) ) {
    CouldNotOpenConfigException ce("Could not copy dump to default: %s", errmsg);
    sqlite3_free(errmsg);
    throw ce;
  }

  // Detach dumped DB, no longer required
  if ( sqlite3_exec(db, SQL_DETACH_DUMPED, NULL, NULL, &errmsg) != SQLITE_OK ) {
    CouldNotOpenConfigException e("Could not detach dump DB in import: %s", errmsg);
    sqlite3_free(errmsg);
    throw e;
  }

  unlink(tmpfile);
  free(tmpfile);
}


/** Begin SQL Transaction.
 * @param ttype transaction type
 */
void
SQLiteConfiguration::transaction_begin(transaction_type_t ttype)
{
  const char *sql = "BEGIN DEFERRED TRANSACTION;";
  if (ttype == TRANSACTION_IMMEDIATE) {
    sql = "BEGIN IMMEDIATE TRANSACTION;";
  } else if (ttype == TRANSACTION_EXCLUSIVE) {
    sql = "BEGIN EXCLUSIVE TRANSACTION;";
  }

  char *errmsg;
  if ( (sqlite3_exec(db, sql, NULL, NULL, &errmsg) != SQLITE_OK) ) {
    throw ConfigurationException("Could not begin transaction (%s)", errmsg);
  }
}

/** Commit SQL Transaction. */
void
SQLiteConfiguration::transaction_commit()
{
  const char *sql = "COMMIT TRANSACTION;";

  char *errmsg;
  if ( (sqlite3_exec(db, sql, NULL, NULL, &errmsg) != SQLITE_OK) ) {
    throw ConfigurationException("Could not commit transaction (%s)", errmsg);
  }
}


/** Rollback SQL Transaction. */
void
SQLiteConfiguration::transaction_rollback()
{
  const char *sql = "ROLLBACK TRANSACTION;";

  char *errmsg;
  if ( (sqlite3_exec(db, sql, NULL, NULL, &errmsg) != SQLITE_OK) ) {
    throw ConfigurationException("Could not rollback transaction (%s)", errmsg);
  }
}

void
SQLiteConfiguration::attach_default(const char *db_file)
{
  char *errmsg;
  char *attach_sql;
  if ( asprintf(&attach_sql, SQL_ATTACH_DEFAULTS, db_file) == -1 ) {
    throw CouldNotOpenConfigException("Could not create attachment SQL");
  }
  if (sqlite3_exec(db, attach_sql, NULL, NULL, &errmsg) != SQLITE_OK) {
    CouldNotOpenConfigException ce(sqlite3_errmsg(db));
    ce.append("Failed to attach default file (%s)",  db_file);
    free(attach_sql);
    throw ce;
  }
  free(attach_sql);
}


void
SQLiteConfiguration::load(const char *file_path)
{
  mutex->lock();

  if (__default_file) free(__default_file);
  if (__default_sql)  free(__default_sql);
  __default_file = NULL;
  __default_sql  = NULL;

  const char *try_paths[] = {__sysconfdir, __userconfdir};
  int try_paths_len = 2;

  char *host_name = NULL;

  if (strcmp(file_path, ":memory:") == 0) {
    __host_file = strdup(":memory:");

    if (sqlite3_open(file_path, &db) != SQLITE_OK) {
      CouldNotOpenConfigException ce(sqlite3_errmsg(db));
      ce.append("Failed to open memory database");
      throw ce;
    }
  } else {
    HostInfo hostinfo;
    if ( asprintf(&host_name, "%s.db", hostinfo.short_name()) == -1 ) {
      host_name = strdup(hostinfo.short_name());
    }

    // determine host file
    // try sysconfdir and userconfdir
    for (int i = 0; i < try_paths_len; ++i) {
      char *path;
      if (asprintf(&path, "%s/%s", try_paths[i], host_name) != -1) {
	if (sqlite3_open(path, &db) == SQLITE_OK) {
	  __host_file = path;
	  break;
	} else {
	  free(path);
	}
      }
    }
  }

  if (__host_file == NULL) {
    CouldNotOpenConfigException ce(sqlite3_errmsg(db));
    ce.append("Failed to open host db (paths)");
    if (host_name) free(host_name);
    throw ce;
  }

  if (file_path == NULL) {
    file_path = "default.sql";
  }

  // determine default file
  if (strcmp(file_path, ":memory:") == 0) {
    try {
      attach_default(":memory:");
    } catch (...) {
      if (host_name)  free(host_name);
      throw;
    }
    __default_file = strdup(":memory:");
  } else {
    if (file_path[0] == '/') {
      // absolute path, take as is
      __default_sql = strdup(file_path);
    } else {
      // try sysconfdir and userconfdir
      for (int i = 0; i < try_paths_len; ++i) {
	char *path;
	if (asprintf(&path, "%s/%s", try_paths[i], file_path) != -1) {
	  if (access(path, F_OK | R_OK) == 0) {
	    __default_sql = path;
	    break;
	  } else {
	    free(path);
	  }
	}
      }
    }

    // Now go for the .db filename

    // generate filename
    char *defaults_db;
    size_t len = strlen(file_path);
    if (fnmatch("*.sql", file_path, FNM_PATHNAME) == 0) {
      defaults_db = (char *)calloc(1, len); // yes, that's one byte less!
      strncpy(defaults_db, file_path, len - 3);
      strcat(defaults_db, "db");
    } else {
      defaults_db = (char *)calloc(1, len + 4);
      strcpy(defaults_db, file_path);
      strcat(defaults_db, ".db");
    }

    if (defaults_db[0] == '/') {
      try {
	attach_default(defaults_db);
	__default_file = defaults_db;
      } catch (...) {
	if (host_name)  free(host_name);
	free(defaults_db);
	throw;
      }
    } else {
      // check directories
      for (int i = 0; i < try_paths_len; ++i) {
	char *path;
	if (asprintf(&path, "%s/%s", try_paths[i], defaults_db) != -1) {
	  try {
	    attach_default(path);
	    __default_file = path;
	    break;
	  } catch (CouldNotOpenConfigException &e) {
	    free(path);
	  }
	}
      }
    }
    free(defaults_db);

    if (__default_file == NULL) {
      if (host_name)  free(host_name);
      throw CouldNotOpenConfigException("Could not create default filename");
    }
  }

  init_dbs();

  if ( __default_sql )  import_default(__default_sql);
  if (host_name)  free(host_name);

  opened = true;

  mutex->unlock();
}


/** Copy all values from the given configuration.
 * All values from the given configuration are copied. Old values are not erased
 * so that the copied values will overwrite existing values, new values are
 * created, but values existent in current config but not in the copie config
 * will remain unchanged.
 * @param copyconf configuration to copy
 */
void
SQLiteConfiguration::copy(Configuration *copyconf)
{
  copyconf->lock();
  transaction_begin();
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
      std::string s = i->get_string();
      set_string(i->path(), s);
    }
  }
  delete i;
  transaction_commit();
  copyconf->unlock();
}


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


std::string
SQLiteConfiguration::get_comment(const char *path)
{
  sqlite3_stmt *stmt;
  const char   *tail;
  std::string   s = "";

  mutex->lock();

  if ( sqlite3_prepare(db, SQL_SELECT_COMMENT, -1, &stmt, &tail) != SQLITE_OK ) {
    mutex->unlock();
    throw ConfigurationException("get_comment: Preparation SQL failed");
  }
  if ( sqlite3_bind_text(stmt, 1, path, -1, NULL) != SQLITE_OK ) {
    mutex->unlock();
    throw ConfigurationException("get_comment: Binding text for path failed (1)");
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


std::string
SQLiteConfiguration::get_default_comment(const char *path)
{
  sqlite3_stmt *stmt;
  const char   *tail;
  std::string   s = "";

  mutex->lock();

  if ( sqlite3_prepare(db, SQL_SELECT_DEFAULT_COMMENT, -1, &stmt, &tail) != SQLITE_OK ) {
    mutex->unlock();
    throw ConfigurationException("get_default_comment: Preparation SQL failed");
  }
  if ( sqlite3_bind_text(stmt, 1, path, -1, NULL) != SQLITE_OK ) {
    mutex->unlock();
    throw ConfigurationException("get_default_comment: Binding text for path failed (1)");
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


bool
SQLiteConfiguration::is_float(const char *path)
{
  return (get_type(path) == "float");
}


bool
SQLiteConfiguration::is_uint(const char *path)
{
  return (get_type(path) == "unsigned int");
}


bool
SQLiteConfiguration::is_int(const char *path)
{
  return (get_type(path) == "int");
}


bool
SQLiteConfiguration::is_bool(const char *path)
{
  return (get_type(path) == "bool");
}


bool
SQLiteConfiguration::is_string(const char *path)
{
  return (get_type(path) == "string");
}


bool
SQLiteConfiguration::is_list(const char *path)
{
  return false;
}


bool
SQLiteConfiguration::is_default(const char *path)
{
  mutex->lock();
  sqlite3_stmt *stmt;
  const char   *tail;
  bool e;

  if ( sqlite3_prepare(db, SQL_SELECT_TYPE, -1, &stmt, &tail) != SQLITE_OK ) {
    mutex->unlock();
    throw ConfigurationException("is_default/prepare", sqlite3_errmsg(db));
  }
  if ( sqlite3_bind_text(stmt, 1, path, -1, NULL) != SQLITE_OK ) {
    mutex->unlock();
    throw ConfigurationException("is_default/bind/path", sqlite3_errmsg(db));
  }
  if ( sqlite3_bind_text(stmt, 2, path, -1, NULL) != SQLITE_OK ) {
    mutex->unlock();
    throw ConfigurationException("is_default/bind/path", sqlite3_errmsg(db));
  }
  e = ( (sqlite3_step(stmt) == SQLITE_ROW) && (sqlite3_column_int(stmt, 1) == 1 ));
  sqlite3_finalize(stmt);

  mutex->unlock();
  return e;
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



std::vector<float>
SQLiteConfiguration::get_floats(const char *path)
{
  throw NotImplementedException("SQLiteConf: list values are not supported");
}


std::vector<unsigned int>
SQLiteConfiguration::get_uints(const char *path)
{
  throw NotImplementedException("SQLiteConf: list values are not supported");
}


std::vector<int>
SQLiteConfiguration::get_ints(const char *path)
{
  throw NotImplementedException("SQLiteConf: list values are not supported");
}

std::vector<bool>
SQLiteConfiguration::get_bools(const char *path)
{
  throw NotImplementedException("SQLiteConf: list values are not supported");
}

std::vector<std::string>
SQLiteConfiguration::get_strings(const char *path)
{
  throw NotImplementedException("SQLiteConf: list values are not supported");
}


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
SQLiteConfiguration::prepare_update(const char *sql,
					  const char *path)
{
  sqlite3_stmt *stmt;
  const char   *tail;

  if ( sqlite3_prepare(db, sql, -1, &stmt, &tail) != SQLITE_OK ) {
    throw ConfigurationException("prepare_update/prepare", sqlite3_errmsg(db));
  }
  if ( sqlite3_bind_text(stmt, 2, path, -1, NULL) != SQLITE_OK ) {
    ConfigurationException ce("prepare_update/bind", sqlite3_errmsg(db));
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


void
SQLiteConfiguration::set_float(const char *path, float f)
{
  sqlite3_stmt *stmt = NULL;

  mutex->lock();

  try {
    stmt = prepare_update(SQL_UPDATE_VALUE, path);
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

  notify_handlers(path);
}


void
SQLiteConfiguration::set_uint(const char *path, unsigned int uint)
{
  sqlite3_stmt *stmt = NULL;

  mutex->lock();

  try {
    stmt = prepare_update(SQL_UPDATE_VALUE, path);
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

  notify_handlers(path);
}


void
SQLiteConfiguration::set_int(const char *path, int i)
{
  sqlite3_stmt *stmt = NULL;

  mutex->lock();

  try {
    stmt = prepare_update(SQL_UPDATE_VALUE, path);
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

  notify_handlers(path);
}


void
SQLiteConfiguration::set_bool(const char *path, bool b)
{
  sqlite3_stmt *stmt = NULL;

  mutex->lock();

  try {
    stmt = prepare_update(SQL_UPDATE_VALUE, path);
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

  notify_handlers(path);
}


void
SQLiteConfiguration::set_string(const char *path,
				const char *s)
{
  sqlite3_stmt *stmt = NULL;

  mutex->lock();

  size_t s_length = strlen(s);

  try {
    stmt = prepare_update(SQL_UPDATE_VALUE, path);
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

  notify_handlers(path);
}


void
SQLiteConfiguration::set_string(const char *path, std::string &s)
{
  set_string(path, s.c_str());
}


void
SQLiteConfiguration::set_floats(const char *path, std::vector<float> &f)
{
  throw NotImplementedException("SQLiteConf: list values are not supported");
}

void
SQLiteConfiguration::set_uints(const char *path, std::vector<unsigned int> &u)
{
  throw NotImplementedException("SQLiteConf: list values are not supported");
}

void
SQLiteConfiguration::set_ints(const char *path, std::vector<int> &i)
{
  throw NotImplementedException("SQLiteConf: list values are not supported");
}

void
SQLiteConfiguration::set_bools(const char *path, std::vector<bool> &b)
{
  throw NotImplementedException("SQLiteConf: list values are not supported");
}

void
SQLiteConfiguration::set_strings(const char *path, std::vector<std::string> &s)
{
  throw NotImplementedException("SQLiteConf: list values are not supported");
}

void
SQLiteConfiguration::set_strings(const char *path, std::vector<const char *> &s)
{
  throw NotImplementedException("SQLiteConf: list values are not supported");
}

void
SQLiteConfiguration::set_comment(const char *path, const char *comment)
{
  sqlite3_stmt *stmt = NULL;

  mutex->lock();

  size_t s_length = strlen(comment);

  try {
    stmt = prepare_update(SQL_UPDATE_COMMENT, path);
    if ( (sqlite3_bind_text(stmt, 1, comment, s_length, SQLITE_STATIC) != SQLITE_OK) ) {
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
    mutex->unlock();
    throw ConfigurationException("set_comment", "Cannot set comment for inexistent path");
  }

  mutex->unlock();

  notify_handlers(path, true);
}


void
SQLiteConfiguration::set_comment(const char *path, std::string &comment)
{
  set_comment(path, comment.c_str());
}


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

  notify_handlers(path);
}


void
SQLiteConfiguration::set_default_float(const char *path, float f)
{
  sqlite3_stmt *stmt = NULL;

  mutex->lock();

  try {
    stmt = prepare_update(SQL_UPDATE_DEFAULT_VALUE, path);
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

  notify_handlers(path);
}


void
SQLiteConfiguration::set_default_uint(const char *path, unsigned int uint)
{
  sqlite3_stmt *stmt = NULL;

  mutex->lock();

  try {
    stmt = prepare_update(SQL_UPDATE_DEFAULT_VALUE, path);
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

  notify_handlers(path);
}


void
SQLiteConfiguration::set_default_int(const char *path, int i)
{
  sqlite3_stmt *stmt = NULL;
  mutex->lock();

  try {
    stmt = prepare_update(SQL_UPDATE_DEFAULT_VALUE, path);
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

  notify_handlers(path);
}


void
SQLiteConfiguration::set_default_bool(const char *path, bool b)
{
  sqlite3_stmt *stmt = NULL;

  mutex->lock();

  try {
    stmt = prepare_update(SQL_UPDATE_DEFAULT_VALUE, path);
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

  notify_handlers(path);
}


void
SQLiteConfiguration::set_default_string(const char *path,
					const char *s)
{
  sqlite3_stmt *stmt = NULL;

  mutex->lock();
  size_t s_length = strlen(s);

  try {
    stmt = prepare_update(SQL_UPDATE_DEFAULT_VALUE, path);
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

  notify_handlers(path);
}


void
SQLiteConfiguration::set_default_string(const char *path, std::string &s)
{
  set_default_string(path, s.c_str());
}


void
SQLiteConfiguration::set_default_comment(const char *path, const char *comment)
{
  sqlite3_stmt *stmt = NULL;

  mutex->lock();
  size_t s_length = strlen(comment);

  try {
    stmt = prepare_update(SQL_UPDATE_DEFAULT_COMMENT, path);
    if ( (sqlite3_bind_text(stmt, 1, comment, s_length, SQLITE_STATIC) != SQLITE_OK) ) {
      ConfigurationException ce("set_default_comment/update/bind", sqlite3_errmsg(db));
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
    mutex->unlock();
    throw ConfigurationException("set_default_comment", "Cannot set comment for inexistent path");
  }

  mutex->unlock();

  notify_handlers(path);
}


void
SQLiteConfiguration::set_default_comment(const char *path, std::string &comment)
{
  set_default_comment(path, comment.c_str());
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

  notify_handlers(path);
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


Configuration::ValueIterator *
SQLiteConfiguration::iterator()
{
  sqlite3_stmt *stmt;
  const char *tail;

  if ( sqlite3_prepare(db, SQL_SELECT_ALL, -1, &stmt, &tail) != SQLITE_OK ) {
    throw ConfigurationException("iterator: Preparation SQL failed");
  }

  return new SQLiteValueIterator(stmt);
}


/** Iterator for all default values.
 * Returns an iterator that can be used to iterate over all default values in
 * the current default configuration. Note that this might return less paths than
 * available, because the values for which no default entry exists are not
 * returned.
 * @return iterator over all default values
 */
Configuration::ValueIterator *
SQLiteConfiguration::iterator_default()
{
  sqlite3_stmt *stmt;
  const char *tail;

  if ( sqlite3_prepare(db, SQL_SELECT_ALL_DEFAULT, -1, &stmt, &tail) != SQLITE_OK ) {
    throw ConfigurationException("iterator_default: Preparation SQL failed");
  }

  return new SQLiteValueIterator(stmt);
}

/** Iterator for all host-specific values.
 * Returns an iterator that can be used to iterate over all host-specific values
 * in the current configuration. Note that this might return less paths than
 * available, because the default values for which no host-specific entry exists
 * are not returned.
 * @return iterator over all host-specific values
 */
Configuration::ValueIterator *
SQLiteConfiguration::iterator_hostspecific()
{
  sqlite3_stmt *stmt;
  const char *tail;

  if ( sqlite3_prepare(db, SQL_SELECT_ALL_HOSTSPECIFIC, -1, &stmt, &tail) != SQLITE_OK ) {
    throw ConfigurationException("iterator_hostspecific: Preparation SQL failed");
  }

  return new SQLiteValueIterator(stmt);
}

/** Iterator for modified values.
 * Returns an iterator that can be used to iterate over all values that have been
 * modified in the default database in the last load (added, erased or changed).
 * @return iterator over all values
 */
SQLiteConfiguration::SQLiteValueIterator *
SQLiteConfiguration::modified_iterator()
{
  sqlite3_stmt *stmt;
  const char *tail;

  if ( sqlite3_prepare(db, SQL_SELECT_MODIFIED_ALL, -1, &stmt, &tail) != SQLITE_OK ) {
    throw ConfigurationException("modified_iterator: Preparation SQL failed");
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

  return new SQLiteValueIterator(stmt, p);
}

/** @class SQLiteConfiguration::SQLiteValueIterator config/sqlite.h
 * SQLite configuration value iterator.
 */


/** Constructor.
 * @param stmt compiled SQLite statement
 * @param p pointer to arbitrary data that is freed (not deleted!) when the iterator
 * is deleted.
 */
SQLiteConfiguration::SQLiteValueIterator::SQLiteValueIterator(::sqlite3_stmt *stmt, void *p)
{
  __stmt = stmt;
  __p = p;
}


/** Destructor. */
SQLiteConfiguration::SQLiteValueIterator::~SQLiteValueIterator()
{
  if ( __stmt != NULL ) {
    sqlite3_finalize(__stmt);
    __stmt = NULL;
  }
  if ( __p != NULL ) {
    free(__p);
  }
}


/* Check if there is another element and advance to this if possible.
 * This advances to the next element, if there is one.
 * @return true, if another element has been reached, false otherwise
 */
bool
SQLiteConfiguration::SQLiteValueIterator::next()
{
  if ( __stmt == NULL) return false;

  if (sqlite3_step(__stmt) == SQLITE_ROW ) {
    return true;
  } else {
    sqlite3_finalize(__stmt);
    __stmt = NULL;
    return false;
  }
}

/** Check if the current element is valid.
 * This is much like the classic end element for iterators. If the iterator is
 * invalid there all subsequent calls to next() shall fail.
 * @return true, if the iterator is still valid, false otherwise
 */
bool
SQLiteConfiguration::SQLiteValueIterator::valid() const
{
  return ( __stmt != NULL);
}


/** Path of value.
 * @return path of value
 */
const char *
SQLiteConfiguration::SQLiteValueIterator::path() const
{
  return (const char *)sqlite3_column_text(__stmt, 0);
}


/** Type of value.
 * @return string representation of value type.
 */
const char *
SQLiteConfiguration::SQLiteValueIterator::type() const
{
  return (const char *)sqlite3_column_text(__stmt, 1);
}


bool
SQLiteConfiguration::SQLiteValueIterator::is_float() const
{
  return (strcmp("float", (const char *)sqlite3_column_text(__stmt, 1)) == 0);
}


bool
SQLiteConfiguration::SQLiteValueIterator::is_uint() const
{
  return (strcmp("unsigned int", (const char *)sqlite3_column_text(__stmt, 1)) == 0);
}

bool
SQLiteConfiguration::SQLiteValueIterator::is_int() const
{
  return (strcmp("int", (const char *)sqlite3_column_text(__stmt, 1)) == 0);
}


bool
SQLiteConfiguration::SQLiteValueIterator::is_bool() const
{
  return (strcmp("bool", (const char *)sqlite3_column_text(__stmt, 1)) == 0);
}


bool
SQLiteConfiguration::SQLiteValueIterator::is_string() const
{
  return (strcmp("string", (const char *)sqlite3_column_text(__stmt, 1)) == 0);
}

bool
SQLiteConfiguration::SQLiteValueIterator::is_list() const
{
  return false;
}


size_t
SQLiteConfiguration::SQLiteValueIterator::get_list_size() const
{
  return 0;
}

bool
SQLiteConfiguration::SQLiteValueIterator::is_default() const
{
  return (sqlite3_column_int(__stmt, 4) == 1);
}


/** Get float value.
 * @return value
 */
float
SQLiteConfiguration::SQLiteValueIterator::get_float() const
{
  return (float)sqlite3_column_double(__stmt, 2);
}


/** Get unsigned int value.
 * @return value
 */
unsigned int
SQLiteConfiguration::SQLiteValueIterator::get_uint() const
{
  int i = sqlite3_column_int(__stmt, 2);
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
SQLiteConfiguration::SQLiteValueIterator::get_int() const
{
  return sqlite3_column_int(__stmt, 2);
}

/** Get bool value.
 * @return value
 */
bool
SQLiteConfiguration::SQLiteValueIterator::get_bool() const
{
  return (sqlite3_column_int(__stmt, 2) != 0);
}

/** Get string value.
 * @return value
 */
std::string
SQLiteConfiguration::SQLiteValueIterator::get_string() const
{
  return (const char *)sqlite3_column_text(__stmt, 2);
}


std::vector<float>
SQLiteConfiguration::SQLiteValueIterator::get_floats() const
{
  throw NotImplementedException("SQLiteConf: list values are not supported");
}

std::vector<unsigned int>
SQLiteConfiguration::SQLiteValueIterator::get_uints() const
{
  throw NotImplementedException("SQLiteConf: list values are not supported");
}

std::vector<int>
SQLiteConfiguration::SQLiteValueIterator::get_ints() const
{
  throw NotImplementedException("SQLiteConf: list values are not supported");
}

std::vector<bool>
SQLiteConfiguration::SQLiteValueIterator::get_bools() const
{
  throw NotImplementedException("SQLiteConf: list values are not supported");
}

std::vector<std::string>
SQLiteConfiguration::SQLiteValueIterator::get_strings() const
{
  throw NotImplementedException("SQLiteConf: list values are not supported");
}

/** Get value as string.
 * @return value
 */
std::string
SQLiteConfiguration::SQLiteValueIterator::get_as_string() const
{
  return (const char *)sqlite3_column_text(__stmt, 2);
}

/** Get comment.
 * @return string comment value
 */
std::string
SQLiteConfiguration::SQLiteValueIterator::get_comment() const
{
  const char *c = (const char *)sqlite3_column_text(__stmt, 3);
  return c ? c : "";
}

/** Get modification type.
 * This can only be called if the iterator has been retrieved via
 * SQLiteConfiguration::modified_iterator(). Otherwise the return value is
 * always and empty string.
 * @return string modification type
 */
std::string
SQLiteConfiguration::SQLiteValueIterator::get_modtype() const
{
  const char *c = (const char *)sqlite3_column_text(__stmt, 4);
  return c ? c : "";
}



/** Get old value (as string).
 * This can only be called if the iterator has been retrieved via
 * SQLiteConfiguration::modified_iterator(). The value is always returned
 * as string, as it is meant for debugging purposes only. Otherwise the
 * return value is always and empty string.
 * @return string modification type
 */
std::string
SQLiteConfiguration::SQLiteValueIterator::get_oldvalue() const
{
  const char *c = (const char *)sqlite3_column_text(__stmt, 5);
  return c ? c : "";
}


} // end namespace fawkes
