#ifndef FAWKES_GOLOGPP_FAWKES_BACKEND_H_
#define FAWKES_GOLOGPP_FAWKES_BACKEND_H_

#include "execution_thread.h"

#include <aspect/clock.h>
#include <aspect/logging.h>
#include <golog++/model/platform_backend.h>

class GologppFawkesBackend : public gologpp::PlatformBackend,
                             public fawkes::ClockAspect,
                             public fawkes::LoggingAspect
{
public:
	GologppFawkesBackend(GologppThread *main_thread);

	virtual void preempt_activity(std::shared_ptr<gologpp::Transition>) override;
	virtual gologpp::Clock::time_point time() const noexcept override;

private:
	virtual void execute_activity(std::shared_ptr<gologpp::Activity>) override;

	GologppThread *main_thread_;
};

#endif
