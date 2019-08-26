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
