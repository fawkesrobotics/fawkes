#include "execution_thread.h"

#include <core/plugin.h>

using namespace fawkes;

class GologppPlugin : public Plugin
{
public:
	explicit GologppPlugin(Configuration *cfg) : Plugin(cfg)
	{
		thread_list.push_back(new GologppThread());
	}
};

PLUGIN_DESCRIPTION("Detect the conveyor belt in a pointcloud")
EXPORT_PLUGIN(GologppPlugin)
