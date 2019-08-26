#include "blackboard.h"

using namespace fawkes;

const std::string GologppBlackboardManager::cfg_prefix = "/plugins/gologpp/blackboard";

GologppBlackboardManager::GologppBlackboardManager()
: BlackBoardInterfaceListener("gologpp_blackboard_manager"),
  Thread("gologpp_blackboard_manager", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(WakeupHook::WAKEUP_HOOK_WORLDSTATE)
{
}

void
GologppBlackboardManager::init()
{
	std::unique_ptr<Configuration::ValueIterator> watch_it(config->search(cfg_prefix + "/watch"));
	while (watch_it->next()) {
		fawkes::Interface *iface = blackboard->open_for_reading<Interface>(
		  config->get_string(std::string(watch_it->path()) + "/id").c_str());
	}
}

void
GologppBlackboardManager::finalize()
{
}

void
GologppBlackboardManager::bb_interface_created(const char *type, const char *id) throw()
{
}

void
GologppBlackboardManager::bb_interface_destroyed(const char *type, const char *id) throw()
{
}

void
GologppBlackboardManager::bb_interface_data_changed(fawkes::Interface *) throw()
{
}

bool
GologppBlackboardManager::bb_interface_message_received(fawkes::Interface *,
                                                        fawkes::Message *) throw()
{
}
