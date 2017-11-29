#ifndef BLACKBOARD_LISTENER_THREAD_H
#define BLACKBOARD_LISTENER_THREAD_H


#include <aspect/blackboard.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <libs/blackboard/interface_observer.h>
#include <libs/blackboard/interface_listener.h>

#include <map>
#include <queue>
#include <memory>

#include "externals/blackboard.h"


using namespace std;
using namespace fawkes;


class BlackboardListenerThread
    : public fawkes::Thread
    , public fawkes::LoggingAspect
    , public fawkes::ConfigurableAspect
    , public fawkes::BlackBoardAspect
    , public fawkes::BlackBoardInterfaceObserver
    , public fawkes::BlackBoardInterfaceListener
{
public:
  BlackboardListenerThread();

  void observe_pattern(const char *type_pattern, const char *id_pattern) noexcept;
  void listen_for_change(Interface *interface) noexcept;

  virtual void bb_interface_created(const char *type, const char *id) noexcept override;
  virtual void bb_interface_destroyed(const char *type, const char *id) noexcept override;
  virtual void bb_interface_data_changed(Interface *interface) noexcept override;

  static BlackboardListenerThread *instance();
  static void cleanup_instance();

private:
  Mutex state_mutex_;

  static BlackboardListenerThread *instance_;

  map<string, fawkes::Interface *> last_iface_of_type_;
  queue<shared_ptr<EclExternalBlackBoard::Event>> iface_events_;
};

#endif // BLACKBOARD_LISTENER_THREAD_H
