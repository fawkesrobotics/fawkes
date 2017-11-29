#include "blackboard_listener_thread.h"

#include <core/threading/mutex_locker.h>

using namespace fawkes;

BlackboardListenerThread *BlackboardListenerThread::instance_ = nullptr;

BlackboardListenerThread::BlackboardListenerThread()
  : Thread("ProtoboardBlackboardManager", Thread::OPMODE_WAITFORWAKEUP)
  , BlackBoardInterfaceListener("eclipse-clp")
{
}


BlackboardListenerThread *BlackboardListenerThread::instance()
{
  if (!instance_)
    instance_ = new BlackboardListenerThread();
  return instance_;
}


void BlackboardListenerThread::cleanup_instance()
{ delete instance_; }


void BlackboardListenerThread::observe_pattern(
    const char *type_pattern, const char *id_pattern) noexcept
{
  MutexLocker lock(&state_mutex_);
  bbio_add_observed_create(type_pattern, id_pattern);
  bbio_add_observed_destroy(type_pattern, id_pattern);
}


void BlackboardListenerThread::listen_for_change(Interface *interface) noexcept
{
  MutexLocker lock(&state_mutex_);
  bbil_add_data_interface(interface);
}


void BlackboardListenerThread::bb_interface_created(const char *type, const char *id) noexcept
{
  MutexLocker lock(&state_mutex_);
  iface_events_.emplace(new EclExternalBlackBoard::Created{type, id});
}


void BlackboardListenerThread::bb_interface_destroyed(const char *type, const char *id) noexcept
{
  MutexLocker lock(&state_mutex_);
  iface_events_.emplace(new EclExternalBlackBoard::Destroyed{type, id});
}


void BlackboardListenerThread::bb_interface_data_changed(Interface *interface) noexcept
{
  MutexLocker lock(&state_mutex_);
  iface_events_.emplace(new EclExternalBlackBoard::Changed{interface});
}

