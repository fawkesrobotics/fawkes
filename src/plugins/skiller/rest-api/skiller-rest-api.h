
/***************************************************************************
 *  clips-executive-rest-api.h -  CLIPS Executive REST API
 *
 *  Created: Fri Mar 16 17:15:34 2018
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
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

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/logging.h>
#include <aspect/webview.h>
#include <aspect/blackboard.h>

#include <webview/rest_api.h>
#include <webview/rest_array.h>

#include "model/Skill.h"
#include "model/SkillCall.h"
#include "model/SkillInfo.h"

namespace fawkes {
	class SkillerDebugInterface;
	class SkillerInterface;
}

class SkillerRestApi
: public fawkes::Thread,
	public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
	public fawkes::BlackBoardAspect,
	public fawkes::WebviewAspect
{
 public:
	SkillerRestApi();
	~SkillerRestApi();

	virtual void init();
	virtual void loop();
	virtual void finalize();

 private:
	WebviewRestArray<SkillInfo> cb_list_skills();

	Skill	cb_get_skill(fawkes::WebviewRestParams& params);

	Skill	cb_exec_skill(const SkillCall &call);

	std::unique_ptr<fawkes::WebviewRestReply>
		cb_stop_skill(fawkes::WebviewRestParams& params);

	void set_and_wait_graph(const char *graph);

 private:
	fawkes::WebviewRestApi        *rest_api_;
	fawkes::SkillerDebugInterface *skdb_if_;
	fawkes::SkillerInterface      *skiller_if_;
};
