
/***************************************************************************
 *  blackboard-rest-api.h -  Blackboard REST API
 *
 *  Created: Mon Mar 26 23:26:40 2018
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
#include <interface/field_iterator.h>
#include <interface/interface_info.h>

#include "model/InterfaceInfo.h"
#include "model/InterfaceData.h"
#include "model/BlackboardGraph.h"

#include <map>
#include <string>
#include <utility>

class BlackboardRestApi
: public fawkes::Thread,
	public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
	public fawkes::BlackBoardAspect,
	public fawkes::WebviewAspect
{
 public:
	BlackboardRestApi();
	~BlackboardRestApi();

	virtual void init();
	virtual void loop();
	virtual void finalize();

 private:
	WebviewRestArray<InterfaceInfo> cb_list_interfaces();

	InterfaceInfo
		cb_get_interface_info(fawkes::WebviewRestParams& params);

	InterfaceData
		cb_get_interface_data(fawkes::WebviewRestParams& params);

	BlackboardGraph cb_get_graph();

	std::vector<std::shared_ptr<InterfaceFieldType>>
		gen_fields(fawkes::InterfaceFieldIterator begin,
		           fawkes::InterfaceFieldIterator end);

	InterfaceInfo gen_interface_info(const fawkes::InterfaceInfo &ii);
	InterfaceData gen_interface_data(fawkes::Interface *iface, bool pretty);

	std::string generate_graph(std::string for_owner = "");

 private:
	fawkes::WebviewRestApi        *rest_api_;

	std::map<std::string, std::pair<std::vector<std::shared_ptr<InterfaceFieldType>>,
	                                std::vector<std::shared_ptr<InterfaceMessageType>>>>
		type_info_cache_;
	

};
