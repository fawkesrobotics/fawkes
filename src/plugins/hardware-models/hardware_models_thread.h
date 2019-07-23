
/***************************************************************************
 *  hardware_models_thread.h - Hardware Models plugin
 *
 *  Created: Sun Mar 24 11:59:44 2019
 *  Copyright  2019  Daniel Habering [daniel@habering.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef _PLUGINS_HARDWARE_MODELS_HARDWARE_MODELS_THREAD_H_
#define _PLUGINS_HARDWARE_MODELS_HARDWARE_MODELS_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/blackboard.h>
#include <aspect/clock.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <blackboard/interface_listener.h>
#include <plugins/clips/aspect/clips_feature.h>
#include <utils/time/time.h>
#include <interfaces/HardwareModelsInterface.h>

#include <clipsmm.h>
#include <memory>

namespace fawkes {
}

class HardwareModelsThread
: public fawkes::Thread,
	public fawkes::LoggingAspect,
	public fawkes::BlackBoardAspect,
	public fawkes::ConfigurableAspect,
	public fawkes::CLIPSFeature,
	public fawkes::CLIPSFeatureAspect,
	public fawkes::BlackBoardInterfaceListener
{
 public:
	HardwareModelsThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

    // for CLIPSFeature
  virtual void clips_context_init(const std::string &env_name,
          fawkes::LockPtr<CLIPS::Environment> &clips);
  virtual void clips_context_destroyed(const std::string &env_name);


	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
    std::map<std::string, fawkes::LockPtr<CLIPS::Environment> >  envs_;

    std::vector<std::string> components_;

    fawkes::HardwareModelsInterface *hm_if_;

    void  clips_add_terminal_state(fawkes::LockPtr<CLIPS::Environment> &clips, const std::string& component, const std::string& state);
    void  clips_add_component(fawkes::LockPtr<CLIPS::Environment> &clips,const std::string& component, const std::string& init_state);
    void  clips_add_edge(fawkes::LockPtr<CLIPS::Environment> &clips,const std::string& component, const std::string& from, const std::string& to, const std::string& trans, const double prob);
    void  clips_add_transition(const std::string& component, const std::string& transition) throw();
};

#endif
