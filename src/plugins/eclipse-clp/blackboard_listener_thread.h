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

#include <eclipseclass.h>

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


  class Event {
  public:
    Event(const std::string &type, const std::string &id)
      : type(type), id(id)
    {}

    virtual ~Event();

    virtual operator EC_word () = 0;

    std::string uid()
    { return type + "::" + id; }

  protected:
    std::string type, id;
  };


  class Created : public Event {
  public:
    using Event::Event;
    virtual operator EC_word ();
  };


  class Destroyed : public Event {
  public:
    using Event::Event;
    virtual operator EC_word ();
  };


  class Changed : public Event {
  public:
    Changed(Interface *interface)
      : Event(interface->type(), interface->id()), interface(interface)
    {}
    virtual operator EC_word ();

  private:
    fawkes::Interface *interface;
  };



  bool event_pending();
  shared_ptr<Event> event_pop();

private:
  Mutex state_mutex_;

  static BlackboardListenerThread *instance_;

  map<string, fawkes::Interface *> last_iface_of_type_;
  queue<shared_ptr<Event>> iface_events_;
};

#endif // BLACKBOARD_LISTENER_THREAD_H
