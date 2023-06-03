
/***************************************************************************
 *  xmlrpc_processor.h - XML-RPC request processor
 *
 *  Created: Sun Aug 30 19:37:50 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef _PLUGINS_XMLRPC_XMLRPC_PROCESSOR_H_
#define _PLUGINS_XMLRPC_XMLRPC_PROCESSOR_H_

#include <map>
#include <memory>
#include <string>

namespace xmlrpc_c {
class registry;
}

namespace fawkes {
class Logger;
class WebReply;
class WebRequest;
} // namespace fawkes

class XmlRpcRequestProcessor
{
public:
	XmlRpcRequestProcessor(fawkes::Logger *logger);
	~XmlRpcRequestProcessor();

	fawkes::WebReply *process_request(const fawkes::WebRequest *request);

	std::shared_ptr<xmlrpc_c::registry> registry();

private:
	fawkes::Logger                     *logger_;
	std::shared_ptr<xmlrpc_c::registry> xmlrpc_registry_;
};

#endif
