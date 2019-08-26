#ifndef FAWKES_GOLOGPP_BLACKBOARD_H_
#define FAWKES_GOLOGPP_BLACKBOARD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>
#include <core/threading/thread.h>
#include <golog++/model/action.h>

class GologppBlackboardManager : public fawkes::BlackBoardInterfaceObserver,
                                 public fawkes::BlackBoardInterfaceListener,
                                 public fawkes::BlackBoardAspect,
                                 public fawkes::ConfigurableAspect,
                                 public fawkes::Thread,
                                 public fawkes::BlockedTimingAspect
{
public:
	GologppBlackboardManager();

	virtual void init() override;
	virtual void finalize() override;

	void sense(gologpp::Activity &sensing_activity);

	virtual void bb_interface_created(const char *type, const char *id) throw() override;
	virtual void bb_interface_destroyed(const char *type, const char *id) throw() override;
	virtual void bb_interface_data_changed(fawkes::Interface *) throw() override;
	virtual bool bb_interface_message_received(fawkes::Interface *,
	                                           fawkes::Message *) throw() override;

	static const std::string cfg_prefix;

private:
	void init_watchers();

	class WatchedInterface
	{
	public:
		WatchedInterface(fawkes::Interface *, gologpp::shared_ptr<gologpp::ExogAction> target_exog);
		void add_field(const std::string &name, gologpp::arity_t param_idx);
		gologpp::shared_ptr<gologpp::ExogEvent> get_event() const;

	private:
		fawkes::Interface *                      iface_;
		gologpp::shared_ptr<gologpp::ExogAction> target_exog_;

		std::unordered_map<std::string, gologpp::arity_t> fields_to_params_;
	};

	std::unordered_map<std::string, WatchedInterface> id_to_exog_action_;
};

#endif
