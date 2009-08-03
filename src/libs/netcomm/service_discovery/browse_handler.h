
/***************************************************************************
 *  browse_handler.h - Avahi browse handler
 *
 *  Created: Wed Nov 08 13:16:47 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __NETCOMM_SERVICE_DISCOVERY_BROWSE_HANDLER_H_
#define __NETCOMM_SERVICE_DISCOVERY_BROWSE_HANDLER_H_

#include <sys/types.h>
#include <sys/socket.h>
#include <stdint.h>

#include <string>
#include <list>

namespace fawkes {

/** @class ServiceBrowseHandler <netcomm/service_discovery/browse_handler.h>
 * Interface for class that process browse results.
 * Implement this class if you want to browse for services on the network.
 * Then register your handler and it will be informed of services that
 * join or leave the network. This is also required for an active search.
 *
 * It is recommended that you read about mDNS, DNS-SD and Avahi.
 *
 * @author Tim Niemueller
 */
class ServiceBrowseHandler
{
 public:
  /** Virtual destructor */
  virtual ~ServiceBrowseHandler() {};

  /** All results have been retrieved.
   * If you read the DNS-SD specs you will see that there is no explicit
   * "not existent" or "end of records" message - it cannot be. But after
   * some time it is assumed that there are no more records. If that is
   * the case this method is called.
   */
  virtual void all_for_now()                                  = 0;

  /** Cache exhausted. */
  virtual void cache_exhausted()                              = 0;

  /** Failed to browse for a given service.
   * @param name name of the service
   * @param type type of the service
   * @param domain domain of the service
   */
  virtual void browse_failed(const char *name,
			     const char *type,
			     const char *domain)              = 0;

  /** A service has been announced on the network.
   * @param name name of the service
   * @param type type of the service
   * @param domain domain of the service
   * @param host_name name of the host that provides the service
   * @param addr pointer to sockaddr struct of appropriate type for address
   * @param addr_size size of addr struct
   * @param port port of the service
   * @param txt list of txt records.
   * @param flags extra flags, see Avahi documentation
   */
  virtual void service_added(const char *name,
			     const char *type,
			     const char *domain,
			     const char *host_name,
			     const struct sockaddr *addr,
			     const socklen_t addr_size,
			     uint16_t port,
			     std::list<std::string> &txt,
			     int flags
			     )                                = 0;

  /** A service has been removed from the network.
   * @param name name of the service
   * @param type type of the service
   * @param domain domain of the service
   */
  virtual void service_removed(const char *name,
			       const char *type,
			       const char *domain)            = 0;

};

} // end namespace fawkes

#endif
