/***************************************************************************
 *  execution_thread.h - Execution thread for Golog++
 *
 *  Created: Mon 26 Aug 2019 CEST 15:38
 *  Copyright  2019  Victor Mataré <matare@fh-aachen.de>
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

#ifndef FAWKES_GOLOGPP_THREAD_H_
#define FAWKES_GOLOGPP_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <blackboard/interface_listener.h>
#include <core/threading/thread.h>
#include <golog++/model/execution.h>

#include <filesystem>
#include <string>

namespace fawkes {
class SkillerInterface;
}

namespace fawkes {
namespace gpp {

class ExogManager;

class GologppThread : public Thread,
                      public LoggingAspect,
                      public BlackBoardAspect,
                      public ConfigurableAspect
{
public:
	GologppThread();

	virtual void init() override;
	virtual void once() override;

	virtual bool prepare_finalize_user() override;

	virtual void finalize() override;

	gologpp::ExecutionContext &gologpp_context();

private:
	std::filesystem::path                 find_prog_file(const std::filesystem::path &spec) const;
	std::unique_ptr<gologpp::Instruction> main_prog_;
	SkillerInterface                     *skiller_if_;
	ExogManager                          *exog_mgr_;
	std::mutex                            run_mutex_;
};

} // namespace gpp
} // namespace fawkes

#endif
