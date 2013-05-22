
/***************************************************************************
 *  procrrd_thread.cpp - Fawkes Proc RRD Thread
 *
 *  Created: Mon Dec 17 12:54:00 2012
 *  Copyright  2012  Bastian Klingen
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

#include "procrrd_thread.h"

#include <utils/time/wait.h>
#include <stdio.h>
#include <dirent.h>
#include <string.h>

using namespace fawkes;

#define PROC_CONF_PREFIX "/plugins/procrrd/processes/"

/** @class ProcRRDThread "procrrd_thread.h"
 * Proc RRD Thread.
 * This thread queries performance data from system every few seconds and
 * writes it to RRD databases. The sampling rate can be set via the config.
 *
 * @author Bastian Klingen
 */

/** Constructor. */
ProcRRDThread::ProcRRDThread()
  : Thread("ProcRRDThread", Thread::OPMODE_CONTINUOUS),
    ConfigurationChangeHandler(PROC_CONF_PREFIX)
{
  set_prepfin_conc_loop(true);
}


/** Destructor. */
ProcRRDThread::~ProcRRDThread()
{
}


void
ProcRRDThread::init()
{
  __samplerate = 10;
  try {
    __samplerate = config->get_uint("/plugins/procrrd/samplerate");
  } catch (Exception &e) {}

  __timewait = new TimeWait(clock, __samplerate * 1000000);
  __netinterface = "wlan0";
  try {
    __netinterface = config->get_string("/plugins/procrrd/netinterface");
  } catch (Exception &e) {}

  __lastcpu = new unsigned long int[11];
  get_cpu(__lastcpu);

  __net_recv_graph = NULL;
  __net_trans_graph = NULL;
  std::vector<RRDDataSource> rrds;
  // /proc/net/dev
  // use data for net_interface
  // Receive bytes
  rrds.push_back(RRDDataSource("net_recv_bytes", RRDDataSource::COUNTER));
  // Receive packets
  rrds.push_back(RRDDataSource("net_recv_packets", RRDDataSource::COUNTER));
  // Receive errs
  rrds.push_back(RRDDataSource("net_recv_errors", RRDDataSource::COUNTER));
  // Transmit bytes
  rrds.push_back(RRDDataSource("net_trans_bytes", RRDDataSource::COUNTER));
  // Transmit packets
  rrds.push_back(RRDDataSource("net_trans_packets", RRDDataSource::COUNTER));
  // Transmit errs
  rrds.push_back(RRDDataSource("net_trans_errors", RRDDataSource::COUNTER));
  __net_rrd = new RRDDefinition("network", rrds);

  try {
    rrd_manager->add_rrd(__net_rrd);
  } catch (Exception &e) {
    finalize();
    throw;
  }

  std::vector<RRDGraphDataDefinition> defs;
  std::vector<RRDGraphElement *> els;

  defs.push_back(RRDGraphDataDefinition("net_recv_bytes", RRDArchive::AVERAGE,
					__net_rrd));
  defs.push_back(RRDGraphDataDefinition("net_recv_packets", RRDArchive::AVERAGE,
					__net_rrd));
  defs.push_back(RRDGraphDataDefinition("net_recv_errors", RRDArchive::AVERAGE,
					__net_rrd));

  els.push_back(new RRDGraphLine("net_recv_bytes", 1, "006400", "Bytes"));
  els.push_back(new RRDGraphGPrint("net_recv_bytes", RRDArchive::LAST,
				   "  Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("net_recv_bytes", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("net_recv_bytes", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphLine("net_recv_packets", 1, "808000", "Packets"));
  els.push_back(new RRDGraphGPrint("net_recv_packets", RRDArchive::LAST,
				   "Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("net_recv_packets", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("net_recv_packets", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphLine("net_recv_errors", 1, "FF0000", "Errors"));
  els.push_back(new RRDGraphGPrint("net_recv_errors", RRDArchive::LAST,
				   " Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("net_recv_errors", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("net_recv_errors", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  __net_recv_graph = new RRDGraphDefinition("network_recv", __net_rrd,
					      "Network Receive", "",
					      defs, els);

  defs.clear();
  els.clear();

  defs.push_back(RRDGraphDataDefinition("net_trans_bytes", RRDArchive::AVERAGE,
					__net_rrd));
  defs.push_back(RRDGraphDataDefinition("net_trans_packets", RRDArchive::AVERAGE,
					__net_rrd));
  defs.push_back(RRDGraphDataDefinition("net_trans_errors", RRDArchive::AVERAGE,
					__net_rrd));

  els.push_back(new RRDGraphLine("net_trans_bytes", 1, "006400", "Bytes"));
  els.push_back(new RRDGraphGPrint("net_trans_bytes", RRDArchive::LAST,
				   "  Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("net_trans_bytes", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("net_trans_bytes", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphLine("net_trans_packets", 1, "808000", "Packets"));
  els.push_back(new RRDGraphGPrint("net_trans_packets", RRDArchive::LAST,
				   "Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("net_trans_packets", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("net_trans_packets", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphLine("net_trans_errors", 1, "FF0000", "Errors"));
  els.push_back(new RRDGraphGPrint("net_trans_errors", RRDArchive::LAST,
				   " Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("net_trans_errors", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("net_trans_errors", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  __net_trans_graph = new RRDGraphDefinition("network_trans", __net_rrd,
					      "Network Transmit", "",
					      defs, els);

  try {
    rrd_manager->add_graph(__net_recv_graph);
    rrd_manager->add_graph(__net_trans_graph);
  } catch (Exception &e) {
    finalize();
    throw;
  }

  std::string procprefix = PROC_CONF_PREFIX;

  try {
    std::string selfid = get_process_id("fawkes"); 
    add_process((procprefix+"fawkes").c_str(), selfid, "fawkes");
  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to add process: Fawkes, exception follows");
    logger->log_warn(name(), e);
    finalize();
    throw;
  }

  Configuration::ValueIterator *i = config->search(procprefix.c_str());
  while (i->next()) {

    if (! i->is_string()) {
      logger->log_warn(name(), "Entry %s is not a string, but of type %s, "
		       "ignoring", i->path(), i->type());
      continue;
    }


    std::string name = i->get_string();
    try {
      std::string pid = get_process_id(name.c_str());
      add_process(i->path(), pid, name);
    } catch (Exception &e) {
      logger->log_warn(this->name(), "Failed to add process: %s, exception follows", name.c_str());
      logger->log_warn(this->name(), e);
      finalize();
      throw;
    }
  }

  std::string p;
  ProcessMap::iterator pi = __processes.begin();
  p = pi->second.name;
  ++pi;
  for (; pi != __processes.end(); ++pi) {
    p += ", "+pi->second.name;
  }

  

  logger->log_info(name(), "ProcRRD logging network interface %s and processes %s with a samplerate of %lu second(s)",
       __netinterface.c_str(), p.c_str(), __samplerate);

  config->add_change_handler(this);
}


void
ProcRRDThread::finalize()
{
  config->rem_change_handler(this);
  delete __timewait;

  rrd_manager->remove_rrd(__net_rrd);

  for (ProcessMap::iterator i = __processes.begin(); i != __processes.end(); ++i) {
    ProcessInfo &info = i->second;
    rrd_manager->remove_rrd(info.rrd);
    delete info.cpu_graph;
    delete info.mem_graph;
    delete info.io_read_graph;
    delete info.io_write_graph;
    delete info.rrd;
  }
  __processes.clear();

  delete __net_recv_graph;
  delete __net_trans_graph;
  delete __net_rrd;
}

void
ProcRRDThread::add_process(const char *path, std::string pid, std::string name)
{
  if (__processes.find(path) != __processes.end()) {
    throw Exception("Process stats for config %s already monitored", path);
  }
  if (pid == "") {
    throw Exception("No PID set.");
  }

  ProcessInfo info;
  info.last_cpu = new unsigned long int[3];
  FILE *file;
  file = fopen(("/proc/"+pid+"/stat").c_str(), "r");
  if(file) {
    // /proc/PID/stat
    // 14th (utime %lu) and 15th (stime %lu) value
    // number of jiffies that the process has executed in user mode and kernel mode
    fscanf(file, "%*d %*s %*c %*d %*d %*d %*d %*d %*u %*u %*u %*u %*u %lu %lu %*d %*d %*d %*d %*d %*d %*u %*u %*d %*u %*u %*u %*u %*u %*u %*u %*u %*u %*u %*u %*u %*u %*d %*d %*u %*u %*u %*d", info.last_cpu+1, info.last_cpu+2);
    // process cpu = utime + stime
    info.last_cpu[0] = info.last_cpu[1] + info.last_cpu[2];
    fclose(file);
  }

  std::vector<RRDDataSource> rrds;
  // totalcpu = sum of /proc/stat line 1
  // usage% = 100 * process cpu / totalcpu
  rrds.push_back(RRDDataSource("cpu_usage", RRDDataSource::GAUGE));
  // Threads: number of currently running threads
  rrds.push_back(RRDDataSource("cpu_threads", RRDDataSource::GAUGE));
  // /proc/PID/status
  // VmPeak: maximum virtual memory space used by the process, in kB (1024 bytes)
  rrds.push_back(RRDDataSource("mem_maxvirt", RRDDataSource::GAUGE));
  // VmSize: current virtual memory space used by the process, in kB (1024 bytes)
  rrds.push_back(RRDDataSource("mem_curvirt", RRDDataSource::GAUGE));
  // VmRss: amount of memory that have been mapped into the process' address space, or its resident set size, in kB (1024 bytes)
  rrds.push_back(RRDDataSource("mem_rss", RRDDataSource::GAUGE));

  // /proc/PID/io
  // rchar: number of bytes the process read, using any read-like system call (from files, pipes, tty...).
  rrds.push_back(RRDDataSource("io_rchar", RRDDataSource::COUNTER));
  // wchar: number of bytes the process wrote using any write-like system call. 
  rrds.push_back(RRDDataSource("io_wchar", RRDDataSource::COUNTER));
  // syscr: number of read-like system call invocations that the process performed. 
  rrds.push_back(RRDDataSource("io_syscr", RRDDataSource::COUNTER));
  // syscw: number of write-like system call invocations that the process performed.
  rrds.push_back(RRDDataSource("io_syscw", RRDDataSource::COUNTER));
  // read_bytes: number of bytes the process directly read from disk. 
  rrds.push_back(RRDDataSource("io_read_bytes", RRDDataSource::COUNTER));
  // write_bytes: number of bytes the process originally dirtied in the page-cache (assuming they will go to disk later). 
  rrds.push_back(RRDDataSource("io_write_bytes", RRDDataSource::COUNTER));
  // cancelled_write_bytes: number of bytes the process "un-dirtied" - e.g. using an "ftruncate" call that truncated pages from the page-cache. 
  rrds.push_back(RRDDataSource("io_cancelled_write", RRDDataSource::COUNTER));

  info.pid = pid;
  info.name = name;

  info.rrd_name = info.name + "_" + info.pid;
  size_t pos = 0;
  size_t at;
  while((at = info.rrd_name.find_first_of(" .-", pos)) != std::string::npos) {
    info.rrd_name.replace(at, 1, "_");
    pos = pos + 1;
  }

  info.rrd = new RRDDefinition(info.rrd_name.c_str(), rrds);

  std::vector<RRDGraphDataDefinition> defs;
  std::vector<RRDGraphElement *> els;

  defs.push_back(RRDGraphDataDefinition("cpu_usage", RRDArchive::AVERAGE,
					info.rrd));
  defs.push_back(RRDGraphDataDefinition("cpu_threads", RRDArchive::AVERAGE,
					info.rrd));

  els.push_back(new RRDGraphLine("cpu_usage", 1, "FF0000", "Usage %"));
  els.push_back(new RRDGraphGPrint("cpu_usage", RRDArchive::LAST,
				   "Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("cpu_usage", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("cpu_usage", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphLine("cpu_threads", 1, "008800", "Threads"));
  els.push_back(new RRDGraphGPrint("cpu_threads", RRDArchive::LAST,
				   "Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("cpu_threads", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("cpu_threads", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  std::string g1name = info.rrd_name + "_cpu";
  std::string g1title = std::string("CPU Usage in % and Number Threads for ") + info.name + "(" + pid + ")";
  info.cpu_graph = new RRDGraphDefinition(g1name.c_str(), info.rrd,
				       g1title.c_str(), "", defs, els);


  defs.clear(); els.clear();
  defs.push_back(RRDGraphDataDefinition("mem_maxvirt_kb", RRDArchive::AVERAGE,
					info.rrd, "mem_maxvirt"));
  defs.push_back(RRDGraphDataDefinition("mem_curvirt_kb", RRDArchive::AVERAGE,
					info.rrd, "mem_curvirt"));
  defs.push_back(RRDGraphDataDefinition("mem_rss_kb", RRDArchive::AVERAGE,
					info.rrd, "mem_rss"));
  defs.push_back(RRDGraphDataDefinition("mem_maxvirt", "mem_maxvirt_kb,1024,*"));
  defs.push_back(RRDGraphDataDefinition("mem_curvirt", "mem_curvirt_kb,1024,*"));
  defs.push_back(RRDGraphDataDefinition("mem_rss", "mem_rss_kb,1024,*"));

  els.push_back(new RRDGraphArea("mem_maxvirt", "3B7AD9", "VmPeak"));
  els.push_back(new RRDGraphGPrint("mem_maxvirt", RRDArchive::LAST,
				   "Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("mem_maxvirt", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("mem_maxvirt", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphArea("mem_curvirt", "6FD1BF", "VmSize"));
  els.push_back(new RRDGraphGPrint("mem_curvirt", RRDArchive::LAST,
				   "Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("mem_curvirt", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("mem_curvirt", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphArea("mem_rss", "0E6E5C", "VmRSS"));
  els.push_back(new RRDGraphGPrint("mem_rss", RRDArchive::LAST,
				   " Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("mem_rss", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("mem_rss", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  std::string g2name = info.rrd_name + "_memory";
  std::string g2title = std::string("Memory Usage for ") + info.name + "(" + pid + ")";
  info.mem_graph = new RRDGraphDefinition(g2name.c_str(), info.rrd,
				       g2title.c_str(), "Bytes", defs, els);


  defs.clear();
  els.clear();
  defs.push_back(RRDGraphDataDefinition("io_rchar", RRDArchive::AVERAGE,
					info.rrd));
  defs.push_back(RRDGraphDataDefinition("io_syscr", RRDArchive::AVERAGE,
					info.rrd));
  defs.push_back(RRDGraphDataDefinition("io_read_bytes", RRDArchive::AVERAGE,
					info.rrd));
  
  els.push_back(new RRDGraphLine("io_rchar", 1, "556B2F", "rchar"));
  els.push_back(new RRDGraphGPrint("io_rchar", RRDArchive::LAST,
				   "Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("io_rchar", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("io_rchar", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphLine("io_syscr", 1, "9ACD32", "syscr"));
  els.push_back(new RRDGraphGPrint("io_syscr", RRDArchive::LAST,
				   "Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("io_syscr", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("io_syscr", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphLine("io_read_bytes", 1, "98FB98", "bytes"));
  els.push_back(new RRDGraphGPrint("io_read_bytes", RRDArchive::LAST,
				   "Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("io_read_bytes", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("io_read_bytes", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  std::string g3name = info.rrd_name + "_io_read";
  std::string g3title = std::string("IO Read Operations for ") + info.name + "(" + pid + ")";
  info.io_read_graph = new RRDGraphDefinition(g3name.c_str(), info.rrd,
				       g3title.c_str(), "Bytes", defs, els);

  defs.clear();
  els.clear();
  defs.push_back(RRDGraphDataDefinition("io_wchar", RRDArchive::AVERAGE,
					info.rrd));
  defs.push_back(RRDGraphDataDefinition("io_syscw", RRDArchive::AVERAGE,
					info.rrd));
  defs.push_back(RRDGraphDataDefinition("io_write_bytes", RRDArchive::AVERAGE,
					info.rrd));
  defs.push_back(RRDGraphDataDefinition("io_cancelled_write", RRDArchive::AVERAGE,
					info.rrd));
  
  els.push_back(new RRDGraphLine("io_wchar", 1, "800000", "wchar"));
  els.push_back(new RRDGraphGPrint("io_wchar", RRDArchive::LAST,
				   "    Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("io_wchar", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("io_wchar", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphLine("io_syscw", 1, "FF0000", "syscw"));
  els.push_back(new RRDGraphGPrint("io_syscw", RRDArchive::LAST,
				   "    Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("io_syscw", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("io_syscw", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphLine("io_write_bytes", 1, "FF8C00", "bytes"));
  els.push_back(new RRDGraphGPrint("io_write_bytes", RRDArchive::LAST,
				   "    Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("io_write_bytes", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("io_write_bytes", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  els.push_back(new RRDGraphLine("io_cancelled_write", 1, "FFA07A", "cancelled"));
  els.push_back(new RRDGraphGPrint("io_cancelled_write", RRDArchive::LAST,
				   "Current\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("io_cancelled_write", RRDArchive::AVERAGE,
				   "Average\\:%8.2lf %s"));
  els.push_back(new RRDGraphGPrint("io_cancelled_write", RRDArchive::MAX,
				   "Maximum\\:%8.2lf %s\\n"));

  std::string g4name = info.rrd_name + "_io_write";
  std::string g4title = std::string("IO Write Operations for ") + info.name + "(" + pid + ")";
  info.io_write_graph = new RRDGraphDefinition(g4name.c_str(), info.rrd,
				       g4title.c_str(), "Bytes", defs, els);
  rrd_manager->add_rrd(info.rrd);
  try {
    rrd_manager->add_graph(info.cpu_graph);
    rrd_manager->add_graph(info.mem_graph);
    rrd_manager->add_graph(info.io_read_graph);
    rrd_manager->add_graph(info.io_write_graph);

    __processes[path] = info;
    logger->log_info(this->name(), "Started monitoring process: %s (PID: %s)",
		     info.name.c_str(), info.pid.c_str());
  } catch (Exception &e) {
    rrd_manager->remove_rrd(info.rrd);
    delete info.cpu_graph;
    delete info.mem_graph;
    delete info.io_read_graph;
    delete info.io_write_graph;
    delete info.rrd;
    throw;
  }
}


void
ProcRRDThread::remove_process(const char *path)
{
  if (__processes.find(path) != __processes.end()) {
    ProcessInfo &info = __processes[path];
    rrd_manager->remove_rrd(info.rrd);
    delete info.cpu_graph;
    delete info.mem_graph;
    delete info.io_read_graph;
    delete info.io_write_graph;
    delete info.rrd;

    logger->log_info(name(), "Stopped monitoring process: %s (PID: %s)",
		     info.name.c_str(), info.pid.c_str());
    __processes.erase(path);
  }
}

std::string
ProcRRDThread::get_process_id(const char *process)
{
	DIR *dir_p;
  FILE *file;
	struct dirent *dir_entry_p;
	std::string result;
	result="";

  // Open /proc/ directory
	dir_p = opendir("/proc/");
  // Reading /proc/ entries
	while(NULL != (dir_entry_p = readdir(dir_p))) {
		// Checking for numbered directories
		if (strspn(dir_entry_p->d_name, "0123456789") == strlen(dir_entry_p->d_name)) {
      std::string f = "/proc/" + std::string(dir_entry_p->d_name) + "/status";
      // open /proc/PID/status
      file = fopen(f.c_str(), "r");
      char name[256];
      if(file) {
        fscanf(file, "Name: %s", name);
        fclose(file);
      }
      if(strcmp(process, name) == 0) {
        result = std::string(dir_entry_p->d_name);
	      closedir(dir_p);
	      return result;
      }
		}
	}
	closedir(dir_p);

	return result;
}

void
ProcRRDThread::get_cpu(unsigned long int* cpus)
{
  FILE *file;
  file = fopen("/proc/stat", "r");
  int i = 0;
  i = fscanf(file, "cpu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu", cpus+1, cpus+2, cpus+3, cpus+4, cpus+5, cpus+6, cpus+7, cpus+8, cpus+9, cpus+10);
  cpus[0] = 0;
  for(int j=1; j<=i; j++)
    cpus[0] += cpus[j];

  fclose(file);
}

void
ProcRRDThread::loop()
{
  __timewait->mark_start();
  FILE *file;
  file = fopen("/proc/net/dev", "r");
  unsigned long long int recv_bytes, recv_packets, recv_errors, trans_bytes, trans_packets, trans_errors;
  recv_bytes = recv_packets = recv_errors = trans_bytes = trans_packets = trans_errors = 0;
  char line[1024];
  while (file && fgets(line, 1024, file) != NULL){
    // sscanf the device to get rid of leading whitespaces
    char dev[100];
    sscanf(line, "%s", dev);

    if (strncmp(dev, (__netinterface+":").c_str(), __netinterface.length()+1) == 0){
      sscanf(line, "%*s %llu %llu %llu %*u %*u %*u %*u %*u %llu %llu %llu %*u %*u %*u %*u %*u", &recv_bytes, &recv_packets, &recv_errors, &trans_bytes, &trans_packets, &trans_errors);
      break;
    }
  }
  fclose(file);
  try {
  	rrd_manager->add_data("network", "N:%llu:%llu:%llu:%llu:%llu:%llu", recv_bytes, recv_packets, recv_errors, trans_bytes, trans_packets, trans_errors);
  } catch (Exception &e) {
  	logger->log_warn(name(), "Failed to update network RRD, exception follows");
  	logger->log_warn(name(), e);
  }

  unsigned long int *curcpu = new unsigned long int[11];
  get_cpu(curcpu);

  for (ProcessMap::iterator i = __processes.begin(); i != __processes.end(); ++i) {
    file = fopen(("/proc/"+i->second.pid+"/stat").c_str(), "r");
    unsigned long int cpu[3];
    if(file) {
      fscanf(file, "%*d %*s %*c %*d %*d %*d %*d %*d %*u %*u %*u %*u %*u %lu %lu", cpu+1, cpu+2);
      cpu[0] = cpu[1] + cpu[2];
      fclose(file);
    }
    file = fopen(("/proc/"+i->second.pid+"/status").c_str(), "r");
    unsigned long long int vmpeak, vmsize, vmrss;
    long int threads = 0;
    vmpeak = vmsize = vmrss = 0;
    char line[128];
    while (file && fgets(line, 128, file) != NULL){
      if (strncmp(line, "VmPeak:", 7) == 0){
        sscanf(line, "VmPeak: %llu", &vmpeak);
      }
      else if (strncmp(line, "VmSize:", 7) == 0){
        sscanf(line, "VmSize: %llu", &vmsize);
      }
      else if (strncmp(line, "VmRSS:", 6) == 0){
        sscanf(line, "VmRSS: %llu", &vmrss);
      }
      else if (strncmp(line, "Threads:", 8) == 0){
        sscanf(line, "Threads: %ld", &threads);
        break;
      }
    }
    fclose(file);
    file = fopen(("/proc/"+i->second.pid+"/io").c_str(), "r");
    unsigned long long int rchar, wchar, syscr, syscw, read_bytes, write_bytes, cancelled_write_bytes;
    rchar = wchar = syscr = syscw = read_bytes = write_bytes = cancelled_write_bytes = 0;
    char line2[128];
    while (file && fgets(line2, 128, file) != NULL){
      if (strncmp(line2, "rchar:", 6) == 0){
        sscanf(line2, "rchar: %llu", &rchar);
      }
      else if (strncmp(line2, "wchar:", 6) == 0){
        sscanf(line2, "wchar: %llu", &wchar);
      }
      else if (strncmp(line2, "syscr:", 6) == 0){
        sscanf(line2, "syscr: %llu", &syscr);
      }
      else if (strncmp(line2, "syscw:", 6) == 0){
        sscanf(line2, "syscw: %llu", &syscw);
      }
      else if (strncmp(line2, "read_bytes:", 11) == 0){
        sscanf(line2, "read_bytes: %llu", &read_bytes);
      }
      else if (strncmp(line2, "write_bytes:", 12) == 0){
        sscanf(line2, "write_bytes: %llu", &write_bytes);
      }
      else if (strncmp(line2, "cancelled_write_bytes:", 22) == 0){
        sscanf(line2, "cancelled_write_bytes: %llu", &cancelled_write_bytes);
        break;
      }
    }
    if(file)
      fclose(file);

    if(curcpu[0] < __lastcpu[0] || curcpu[1] < __lastcpu[1] || curcpu[2] < __lastcpu[2] || curcpu[3] < __lastcpu[3] || curcpu[4] < __lastcpu[4] || curcpu[5] < __lastcpu[5] || curcpu[6] < __lastcpu[6] || curcpu[7] < __lastcpu[7] || curcpu[8] < __lastcpu[8] || curcpu[9] < __lastcpu[9] || curcpu[10] < __lastcpu[10] || cpu[0] < i->second.last_cpu[0] || cpu[1] < i->second.last_cpu[1] || cpu[2] < i->second.last_cpu[2]) {
      // value rollover for atleast one value, ignore cpu data
      try {
      	rrd_manager->add_data(i->second.rrd_name.c_str(), "N:U:%ld:%llu:%llu:%llu:%llu:%llu:%llu:%llu:%llu:%llu:%llu", threads, vmpeak, vmsize, vmrss, rchar, wchar, syscr, syscw, read_bytes, write_bytes, cancelled_write_bytes);
      } catch (Exception &e) {
      	logger->log_warn(name(), "Failed to update %s RRD, exception follows", i->second.rrd_name.c_str());
      	logger->log_warn(name(), e);
      }
    }
    else {
      double cpuuse = 100.0 * (double)(cpu[0] - i->second.last_cpu[0]) / (double)(curcpu[0] - __lastcpu[0]);
      try {
      	rrd_manager->add_data(i->second.rrd_name.c_str(), "N:%f:%ld:%llu:%llu:%llu:%llu:%llu:%llu:%llu:%llu:%llu:%llu", cpuuse, threads, vmpeak, vmsize, vmrss, rchar, wchar, syscr, syscw, read_bytes, write_bytes, cancelled_write_bytes);
      } catch (Exception &e) {
      	logger->log_warn(name(), "Failed to update %s RRD, exception follows", i->second.rrd_name.c_str());
      	logger->log_warn(name(), e);
      }
    }

    i->second.last_cpu[0] = cpu[0];
  }

  __lastcpu = curcpu;

  __timewait->wait_systime();
}

void
ProcRRDThread::config_tag_changed(const char *new_tag)
{
  // ignored
}


void
ProcRRDThread::config_value_changed(const Configuration::ValueIterator *v)
{
  if (v->is_string()) {
    remove_process(v->path());
    std::string name = v->get_string();
    std::string pid = get_process_id(name.c_str());
    add_process(v->path(), pid, name);
  } else {
    logger->log_warn(name(), "Non-string value at %s, ignoring", v->path());
  }
}

void
ProcRRDThread::config_comment_changed(const Configuration::ValueIterator *v)
{
}

void
ProcRRDThread::config_value_erased(const char *path)
{
		   
  remove_process(path);
}
