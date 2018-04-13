
/***************************************************************************
 *  mongorrd_thread.cpp - MongoDB RRD Thread
 *
 *  Created: Sat Jan 15 18:42:39 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "mongorrd_thread.h"

#include <utils/time/wait.h>

// from MongoDB
#include <mongo/client/dbclient.h>

using namespace mongo;
using namespace fawkes;

#define DB_CONF_PREFIX "/plugins/mongorrd/databases/"

/** @class MongoRRDThread "mongorrd_thread.h"
 * MongoDB RRD Thread.
 * This thread queries performance data from MongoDB every 10 seconds and
 * writes it to RRD databases.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
MongoRRDThread::MongoRRDThread()
  : Thread("MongoRRDThread", Thread::OPMODE_CONTINUOUS),
    MongoDBAspect("default"),
    ConfigurationChangeHandler(DB_CONF_PREFIX)
{
  set_prepfin_conc_loop(true);
}


/** Destructor. */
MongoRRDThread::~MongoRRDThread()
{
}


void
MongoRRDThread::init()
{
  __timewait = new TimeWait(clock, 10 * 1000000);

  __opcounters_graph = NULL;
  __memory_graph = NULL;
  __indexes_graph = NULL;

  std::vector<RRDDataSource> rrds;
  rrds.push_back(RRDDataSource("insert",  RRDDataSource::COUNTER));
  rrds.push_back(RRDDataSource("query",   RRDDataSource::COUNTER));
  rrds.push_back(RRDDataSource("update",  RRDDataSource::COUNTER));
  rrds.push_back(RRDDataSource("delete",  RRDDataSource::COUNTER));
  rrds.push_back(RRDDataSource("getmore", RRDDataSource::COUNTER));
  rrds.push_back(RRDDataSource("command", RRDDataSource::COUNTER));
  __opcounters_rrd = new RRDDefinition("opcounters", rrds);

  rrds.clear();
  rrds.push_back(RRDDataSource("resident", RRDDataSource::GAUGE));
  rrds.push_back(RRDDataSource("virtual", RRDDataSource::GAUGE));
  rrds.push_back(RRDDataSource("mapped", RRDDataSource::GAUGE));
  __memory_rrd = new RRDDefinition("memory", rrds);

  rrds.clear();
  rrds.push_back(RRDDataSource("accesses", RRDDataSource::COUNTER));
  rrds.push_back(RRDDataSource("hits", RRDDataSource::COUNTER));
  rrds.push_back(RRDDataSource("misses", RRDDataSource::COUNTER));
  rrds.push_back(RRDDataSource("resets", RRDDataSource::COUNTER));
  __indexes_rrd = new RRDDefinition("indexes", rrds);

  rrds.clear();
  rrds.push_back(RRDDataSource("locktime", RRDDataSource::COUNTER));
  __locks_rrd = new RRDDefinition("locks", rrds);


  try {
    rrd_manager->add_rrd(__opcounters_rrd);
    rrd_manager->add_rrd(__memory_rrd);
    rrd_manager->add_rrd(__indexes_rrd);
    rrd_manager->add_rrd(__locks_rrd);
  } catch (Exception &e) {
    finalize();
    throw;
  }


  std::vector<RRDGraphDataDefinition> defs;
  std::vector<RRDGraphElement *> els;

  defs.push_back(RRDGraphDataDefinition("insert", RRDArchive::AVERAGE,
					__opcounters_rrd));
  defs.push_back(RRDGraphDataDefinition("query", RRDArchive::AVERAGE,
					__opcounters_rrd));
  defs.push_back(RRDGraphDataDefinition("update", RRDArchive::AVERAGE,
					__opcounters_rrd));
  defs.push_back(RRDGraphDataDefinition("delete", RRDArchive::AVERAGE,
					__opcounters_rrd));
  defs.push_back(RRDGraphDataDefinition("getmore", RRDArchive::AVERAGE,
					__opcounters_rrd));
  defs.push_back(RRDGraphDataDefinition("command", RRDArchive::AVERAGE,
					__opcounters_rrd));
  
  els.push_back(new RRDGraphLine("insert", 1, "FF7200", "Inserts"));
  els.push_back(new RRDGraphGPrint("insert", RRDArchive::LAST,
				   " Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("insert", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("insert", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphLine("query", 1, "503001", "Queries"));
  els.push_back(new RRDGraphGPrint("query", RRDArchive::LAST,
				   " Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("query", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("query", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphLine("update", 1, "EDAC00", "Updates"));
  els.push_back(new RRDGraphGPrint("update", RRDArchive::LAST,
				   " Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("update", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("update", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphLine("delete", 1, "506101", "Deletes"));
  els.push_back(new RRDGraphGPrint("delete", RRDArchive::LAST,
				   " Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("delete", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("delete", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphLine("getmore", 1, "0CCCCC", "Getmores"));
  els.push_back(new RRDGraphGPrint("getmore", RRDArchive::LAST,
				   "Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("getmore", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("getmore", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphLine("command", 1, "53CA05", "Commands"));
  els.push_back(new RRDGraphGPrint("command", RRDArchive::LAST,
				   "Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("command", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("command", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  __opcounters_graph = new RRDGraphDefinition("opcounters", __opcounters_rrd,
					      "MongoDB Op Counters", "Ops/sec",
					      defs, els);


  defs.clear(); els.clear();
  defs.push_back(RRDGraphDataDefinition("rawresident", RRDArchive::AVERAGE,
					__memory_rrd, "resident"));
  defs.push_back(RRDGraphDataDefinition("rawvirtual", RRDArchive::AVERAGE,
					__memory_rrd, "virtual"));
  defs.push_back(RRDGraphDataDefinition("rawmapped", RRDArchive::AVERAGE,
					__memory_rrd, "mapped"));
  defs.push_back(RRDGraphDataDefinition("resident", "rawresident,1048576,*"));
  defs.push_back(RRDGraphDataDefinition("virtual", "rawvirtual,1048576,*"));
  defs.push_back(RRDGraphDataDefinition("mapped", "rawmapped,1048576,*"));

  els.push_back(new RRDGraphArea("virtual", "3B7AD9", "Virtual"));
  els.push_back(new RRDGraphGPrint("virtual", RRDArchive::LAST,
				   " Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("virtual", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("virtual", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphArea("mapped", "6FD1BF", "Mapped"));
  els.push_back(new RRDGraphGPrint("mapped", RRDArchive::LAST,
				   "  Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("mapped", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("mapped", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphArea("resident", "0E6E5C", "Resident"));
  els.push_back(new RRDGraphGPrint("resident", RRDArchive::LAST,
				   "Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("resident", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("resident", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  __memory_graph = new RRDGraphDefinition("memory", __memory_rrd,
					  "MongoDB Memory Usage", "MB",
					  defs, els);

  defs.clear(); els.clear();
  defs.push_back(RRDGraphDataDefinition("accesses", RRDArchive::AVERAGE,
					__indexes_rrd));
  defs.push_back(RRDGraphDataDefinition("hits", RRDArchive::AVERAGE,
					__indexes_rrd));
  defs.push_back(RRDGraphDataDefinition("misses", RRDArchive::AVERAGE,
					__indexes_rrd));
  defs.push_back(RRDGraphDataDefinition("resets", RRDArchive::AVERAGE,
					__indexes_rrd));
  
  els.push_back(new RRDGraphLine("accesses", 1, "FF7200", "Accesses"));
  els.push_back(new RRDGraphGPrint("accesses", RRDArchive::LAST,
				   "Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("accesses", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("accesses", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphLine("hits", 1, "503001", "Hits"));
  els.push_back(new RRDGraphGPrint("hits", RRDArchive::LAST,
				   "    Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("hits", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("hits", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphLine("misses", 1, "EDAC00", "Misses"));
  els.push_back(new RRDGraphGPrint("misses", RRDArchive::LAST,
				   "  Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("misses", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("misses", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphLine("resets", 1, "506101", "Resets"));
  els.push_back(new RRDGraphGPrint("resets", RRDArchive::LAST,
				   "  Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("resets", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("resets", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  __indexes_graph = new RRDGraphDefinition("indexes", __indexes_rrd,
					    "MongoDB Indexes", "",
					    defs, els);

  try {
    rrd_manager->add_graph(__opcounters_graph);
    rrd_manager->add_graph(__memory_graph);
    rrd_manager->add_graph(__indexes_graph);
  } catch (Exception &e) {
    finalize();
    throw;
  }

  // Add DB Stats
  std::string dbprefix = DB_CONF_PREFIX;

  Configuration::ValueIterator *i = config->search(dbprefix.c_str());
  while (i->next()) {

    if (! i->is_string()) {
      logger->log_warn(name(), "Entry %s is not a string, but of type %s, "
		       "ignoring", i->path(), i->type());
      continue;
    }

    std::string dbname = i->get_string();
    if (dbname.find(".") != std::string::npos) {
      logger->log_warn(name(), "Database name %s contains dot, ignoring",
		       dbname.c_str());
      continue;
    }

    try {
      add_dbstats(i->path(), dbname);
    } catch (Exception &e) {
      finalize();
      throw;
    }
  }

  config->add_change_handler(this);
}


void
MongoRRDThread::finalize()
{
  config->rem_change_handler(this);
  delete __timewait;

  rrd_manager->remove_rrd(__opcounters_rrd);
  rrd_manager->remove_rrd(__memory_rrd);
  rrd_manager->remove_rrd(__indexes_rrd);
  rrd_manager->remove_rrd(__locks_rrd);

  for (DbStatsMap::iterator i = __dbstats.begin(); i != __dbstats.end(); ++i) {
    DbStatsInfo &info = i->second;
    rrd_manager->remove_rrd(info.rrd);
    delete info.graph1;
    delete info.graph2;
    delete info.graph3;
    delete info.rrd;
  }
  __dbstats.clear();

  delete __opcounters_graph;
  delete __memory_graph;
  delete __indexes_graph;

  delete __opcounters_rrd;
  delete __memory_rrd;
  delete __indexes_rrd;
  delete __locks_rrd;
}

void
MongoRRDThread::add_dbstats(const char *path, std::string dbname)
{
  if (__dbstats.find(path) != __dbstats.end()) {
    throw Exception("Database stats for config %s already monitored", path);
  }

  DbStatsInfo info;

  std::vector<RRDDataSource> rrds;
  rrds.push_back(RRDDataSource("collections", RRDDataSource::GAUGE));
  rrds.push_back(RRDDataSource("objects", RRDDataSource::GAUGE));
  rrds.push_back(RRDDataSource("avgObjSize", RRDDataSource::GAUGE));
  rrds.push_back(RRDDataSource("dataSize", RRDDataSource::GAUGE));
  rrds.push_back(RRDDataSource("storageSize", RRDDataSource::GAUGE));
  rrds.push_back(RRDDataSource("numExtents", RRDDataSource::GAUGE));
  rrds.push_back(RRDDataSource("indexes", RRDDataSource::GAUGE));
  rrds.push_back(RRDDataSource("indexSize", RRDDataSource::GAUGE));
  rrds.push_back(RRDDataSource("fileSize", RRDDataSource::GAUGE));

  info.db_name = dbname;
  info.rrd_name = std::string("dbstats_")+dbname;
  info.rrd = new RRDDefinition(info.rrd_name.c_str(), rrds);

  std::vector<RRDGraphDataDefinition> defs;
  std::vector<RRDGraphElement *> els;

  defs.push_back(RRDGraphDataDefinition("collections", RRDArchive::AVERAGE,
					info.rrd));
  defs.push_back(RRDGraphDataDefinition("indexes", RRDArchive::AVERAGE,
					info.rrd));
  defs.push_back(RRDGraphDataDefinition("numExtents", RRDArchive::AVERAGE,
					info.rrd));

  els.push_back(new RRDGraphLine("collections", 1, "FF7200", "Collections"));
  els.push_back(new RRDGraphGPrint("collections", RRDArchive::LAST,
				   "Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("collections", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("collections", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphLine("indexes", 1, "EDAC00", "Indexes"));
  els.push_back(new RRDGraphGPrint("indexes", RRDArchive::LAST,
				   "    Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("indexes", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("indexes", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphLine("numExtents", 1, "506101", "Extents"));
  els.push_back(new RRDGraphGPrint("numExtents", RRDArchive::LAST,
				   "    Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("numExtents", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("numExtents", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  std::string g1name = info.rrd_name + "_collindext";
  std::string g1title = std::string("MongoDB Collections, Indexes, Extents for ")
    + dbname;
  info.graph1 = new RRDGraphDefinition(g1name.c_str(), info.rrd,
				       g1title.c_str(), "", defs, els);


  defs.clear(); els.clear();
  defs.push_back(RRDGraphDataDefinition("objects", RRDArchive::AVERAGE,
					info.rrd));

  els.push_back(new RRDGraphLine("objects", 1, "FF7200", "Objects"));
  els.push_back(new RRDGraphGPrint("objects", RRDArchive::LAST,
				   " Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("objects", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("objects", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  std::string g2name = info.rrd_name + "_objects";
  std::string g2title = std::string("MongoDB Objects for ") + dbname;
  info.graph2 = new RRDGraphDefinition(g2name.c_str(), info.rrd,
				       g2title.c_str(), "", defs, els);


  defs.clear(); els.clear();
  defs.push_back(RRDGraphDataDefinition("avgObjSize", RRDArchive::AVERAGE,
					info.rrd));
  defs.push_back(RRDGraphDataDefinition("dataSize", RRDArchive::AVERAGE,
					info.rrd));
  defs.push_back(RRDGraphDataDefinition("storageSize", RRDArchive::AVERAGE,
					info.rrd));
  defs.push_back(RRDGraphDataDefinition("indexSize", RRDArchive::AVERAGE,
					info.rrd));
  defs.push_back(RRDGraphDataDefinition("fileSize", RRDArchive::AVERAGE,
					info.rrd));
  
  els.push_back(new RRDGraphLine("avgObjSize", 1, "FF7200", "Avg Obj Sz"));
  els.push_back(new RRDGraphGPrint("avgObjSize", RRDArchive::LAST,
				   "Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("avgObjSize", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("avgObjSize", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphLine("dataSize", 1, "503001", "Data"));
  els.push_back(new RRDGraphGPrint("dataSize", RRDArchive::LAST,
				   "      Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("dataSize", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("dataSize", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphLine("storageSize", 1, "EDAC00", "Storage"));
  els.push_back(new RRDGraphGPrint("storageSize", RRDArchive::LAST,
				   "   Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("storageSize", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("storageSize", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphLine("indexSize", 1, "506101", "Index"));
  els.push_back(new RRDGraphGPrint("indexSize", RRDArchive::LAST,
				   "     Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("indexSize", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("indexSize", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphLine("fileSize", 1, "0CCCCC", "File"));
  els.push_back(new RRDGraphGPrint("fileSize", RRDArchive::LAST,
				   "      Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("fileSize", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("fileSize", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  std::string g3name = info.rrd_name + "_sizes";
  std::string g3title = std::string("MongoDB Sizes for ") + dbname;
  info.graph3 = new RRDGraphDefinition(g3name.c_str(), info.rrd,
				       g3title.c_str(), "Mem", defs, els);


  rrd_manager->add_rrd(info.rrd);
  try {
    rrd_manager->add_graph(info.graph1);
    rrd_manager->add_graph(info.graph2);
    rrd_manager->add_graph(info.graph3);

    __dbstats[dbname] = info;
    logger->log_info(name(), "Started monitoring MongoDB %s",
		     info.db_name.c_str());
  } catch (Exception &e) {
    rrd_manager->remove_rrd(info.rrd);
    delete info.graph1;
    delete info.graph2;
    delete info.graph3;
    delete info.rrd;
    throw;
  }
}


void
MongoRRDThread::remove_dbstats(const char *path)
{
  if (__dbstats.find(path) != __dbstats.end()) {
    DbStatsInfo &info = __dbstats[path];
    rrd_manager->remove_rrd(info.rrd);
    delete info.graph1;
    delete info.graph2;
    delete info.graph3;
    delete info.rrd;

    logger->log_info(name(), "Stopped monitoring MongoDB %s",
		     info.db_name.c_str());
    __dbstats.erase(path);
  }
}


void
MongoRRDThread::loop()
{
  __timewait->mark_start();

  try {
    BSONObj reply;
    if (mongodb_client->simpleCommand("admin", &reply, "serverStatus")) {
      BSONObj opcounters = reply["opcounters"].Obj();
      int insert, query, update, del, getmore, command;
      insert  = opcounters["insert"].Int();
      query   = opcounters["query"].Int();
      update  = opcounters["update"].Int();
      del     = opcounters["delete"].Int();
      getmore = opcounters["getmore"].Int();
      command = opcounters["command"].Int();

      try {
	rrd_manager->add_data("opcounters", "N:%i:%i:%i:%i:%i:%i", insert, query,
			      update, del, getmore, command);
      } catch (Exception &e) {
	logger->log_warn(name(), "Failed to update opcounters RRD, "
			 "exception follows");
	logger->log_warn(name(), e);
      }

      BSONObj mem = reply["mem"].Obj();
      int resident, virtmem, mapped;
      resident = mem["resident"].Int();
      virtmem  = mem["virtual"].Int();
      mapped   = mem["mapped"].Int();

      try {
	rrd_manager->add_data("memory", "N:%i:%i:%i", resident, virtmem, mapped);
      } catch (Exception &e) {
	logger->log_warn(name(), "Failed to update memory RRD, exception follows");
	logger->log_warn(name(), e);
      }


      BSONObj indexc = reply["indexCounters"].Obj()["btree"].Obj();
      int accesses, hits, misses, resets;
      accesses = indexc["accesses"].Int();
      hits     = indexc["hits"].Int();
      misses   = indexc["misses"].Int();
      resets   = indexc["resets"].Int();

      try {
	rrd_manager->add_data("indexes", "N:%i:%i:%i:%i",
			      accesses, hits, misses, resets);
      } catch (Exception &e) {
	logger->log_warn(name(), "Failed to update indexes RRD, "
			 "exception follows");
	logger->log_warn(name(), e);
      }

      for (DbStatsMap::iterator i = __dbstats.begin(); i != __dbstats.end(); ++i) {
	BSONObj dbstats;
	if (mongodb_client->simpleCommand(i->second.db_name, &dbstats, "dbStats"))
	{
	  long int collections, objects, numExtents, indexes, dataSize,
	    storageSize, indexSize, fileSize;
	  double avgObjSize;

	  try {
	    collections = dbstats["collections"].numberLong();
	    objects     = dbstats["objects"].numberLong();
	    avgObjSize  = dbstats["avgObjSize"].Double();
	    dataSize    = dbstats["dataSize"].numberLong();
	    storageSize = dbstats["storageSize"].numberLong();
	    numExtents  = dbstats["numExtents"].numberLong();
	    indexes     = dbstats["indexes"].numberLong();
	    indexSize   = dbstats["indexSize"].numberLong();
	    fileSize    = dbstats["fileSize"].numberLong();

	    try {
	      rrd_manager->add_data(i->second.rrd_name.c_str(),
				    "N:%li:%li:%f:%li:%li:%li:%li:%li:%li", collections,
				    objects, avgObjSize, dataSize, storageSize,
				    numExtents, indexes, indexSize, fileSize);
	    } catch (Exception &e) {
	      logger->log_warn(name(), "Failed to update dbstates RRD for "
			       "%s exception follows", i->second.db_name.c_str());
	      logger->log_warn(name(), e);
	    }

	  } catch (mongo::MsgAssertionException &ue) {
	    logger->log_warn(name(), "Failed to update MongoDB RRD for database "
			     "%s: %s", i->second.db_name.c_str(), ue.what());
	  } catch (mongo::UserException &ue) {
	    logger->log_warn(name(), "Failed to update MongoDB RRD for database "
			     "%s: %s", i->second.db_name.c_str(), ue.what());
	  }
	} else {
	  logger->log_warn(name(), "Failed to retrieve db stats for %s: %s",
			   i->second.db_name.c_str(),
			   mongodb_client->getLastError().c_str());
	}
      }

      //double locktime = reply["globalLock"].Obj()["lockTime"].Number();

    } else {
      logger->log_warn(name(), "Failed to retrieve server status: %s",
		       mongodb_client->getLastError().c_str());
    }

  } catch (mongo::UserException &e) {
    logger->log_warn(name(), "Failed to update MongoDB RRD: %s", e.what());
  }

  __timewait->wait_systime();
}

void
MongoRRDThread::config_tag_changed(const char *new_tag)
{
  // ignored
}


void
MongoRRDThread::config_value_changed(const Configuration::ValueIterator *v)
{
  if (v->is_string()) {
    remove_dbstats(v->path());
    add_dbstats(v->path(), v->get_string());
  } else {
    logger->log_warn(name(), "Non-string value at %s, ignoring", v->path());
  }
}

void
MongoRRDThread::config_comment_changed(const Configuration::ValueIterator *v)
{
}

void
MongoRRDThread::config_value_erased(const char *path)
{
		   
  remove_dbstats(path);
}
