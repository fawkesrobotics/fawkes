#ifndef FAWKES_GOLOGPP_THREAD_H_
#define FAWKES_GOLOGPP_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <model/execution.h>

class GologppThread : public fawkes::Thread,
                      public fawkes::LoggingAspect,
                      public fawkes::ConfigurableAspect
{
public:
	GologppThread();

	virtual void init() override;
	virtual void once() override;
	virtual void finalize() override;

private:
	std::unique_ptr<gologpp::Expression> main_prog_;
};

#endif
