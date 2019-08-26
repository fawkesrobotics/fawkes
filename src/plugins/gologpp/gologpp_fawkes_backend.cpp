#include "gologpp_fawkes_backend.h"

using namespace gologpp;
using namespace fawkes;

GologppFawkesBackend::GologppFawkesBackend(GologppThread *main_thread) : main_thread_(main_thread)
{
}

void
GologppFawkesBackend::preempt_activity(shared_ptr<Transition> t)
{
}

gologpp::Clock::time_point
GologppFawkesBackend::time() const noexcept
{
	return gologpp::Clock::time_point{
	  gologpp::Clock::duration{clock->now().in_sec() / gologpp::Clock::duration::period::den}};
}

void
GologppFawkesBackend::execute_activity(shared_ptr<Activity> a)
{
}
