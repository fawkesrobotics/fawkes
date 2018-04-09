
/***************************************************************************
 *  clips-rest-api.h -  CLIPS REST API
 *
 *  Created: Sat Mar 31 01:35:21 2018
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
#include <aspect/logging.h>
#include <aspect/webview.h>
#include <plugins/clips/aspect/clips_manager.h>

#include <webview/rest_api.h>
#include <webview/rest_array.h>

#include <core/utils/lockptr.h>
#include <clipsmm/fact.h>

#include "model/Fact.h"
#include "model/Environment.h"

namespace CLIPS {
	class Environment;
}

class ClipsRestApi
: public fawkes::Thread,
  public fawkes::LoggingAspect,
	public fawkes::WebviewAspect,
  public fawkes::CLIPSManagerAspect
{
 public:
	ClipsRestApi();
	~ClipsRestApi();

	virtual void init();
	virtual void loop();
	virtual void finalize();

 private:
	WebviewRestArray<Environment> cb_list_environments();
	WebviewRestArray<Fact>
		cb_get_facts(fawkes::WebviewRestParams& params);

	Fact gen_fact(fawkes::LockPtr<CLIPS::Environment>& clips,
	              CLIPS::Fact::pointer &fact, bool formatted);


 private:
	fawkes::WebviewRestApi *rest_api_;
	
};
