#include "gologpp_fawkes_backend.h"
#include "gologpp_thread.h"

#include <golog++/model/procedural.h>
#include <golog++/parser/parser.h>
#include <golog++/semantics/readylog/execution.h>

using namespace fawkes;

const std::string cfg_prefix("/plugins/gologpp");

GologppThread::GologppThread() : fawkes::Thread("gologpp_agent", Thread::OPMODE_WAITFORWAKEUP)
{
}

void
GologppThread::init()
{
	std::string prog_file = this->config->get_string(cfg_prefix + "/program_path");
	if (prog_file[0] != '/')
		prog_file = SRCDIR "/" + prog_file;

	logger->log_info(name(), "Parsing %s...", prog_file.c_str());
	main_prog_ = gologpp::parser::parse_file(prog_file);
	logger->log_info(name(), "... parsing done");

	logger->log_info(name(), "Initializing ReadyLog context...");
	gologpp::ReadylogContext::init({}, std::make_unique<GologppFawkesBackend>(this));
	logger->log_info(name(), "... initialization done");
}

void
GologppThread::once()
{
	gologpp::ReadylogContext::instance().run(
	  gologpp::Block{new gologpp::Scope{gologpp::global_scope()}, {main_prog_.release()}});
}

void
GologppThread::finalize()
{
	gologpp::ReadylogContext::shutdown();
}
