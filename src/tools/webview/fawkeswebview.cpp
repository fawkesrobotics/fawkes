
/***************************************************************************
 *  fawkeswebview.cpp - Fawkes Webview
 *
 *  Generated: Fri Oct 03 17:42:07 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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


#include <fcgi_config.h>
#include <string>
#include <map>
#include <stdlib.h>
#include <unistd.h>

#include <blackboard/remote.h>
#include <interface/interface.h>
#include <interface/interface_info.h>
#include <utils/system/signal.h>

#include <fcgi_stdio.h>
#include <fcgiapp.h>

using namespace fawkes;

extern char **environ;

/** Fawkes webview class.
 * This is the base class for the Fawkes webview FastCGI application.
 * @author Tim Niemueller
 */
class FawkesWebviewMain : public SignalHandler
{
 public:
  /** Construtor. */
  FawkesWebviewMain()
  {
    count = 0;
    bb = NULL;
    try_open_bb();
  }

  /** Destructor. */
  ~FawkesWebviewMain()
  {
    close_bb();
  }


  /** Run the web application. */
  void run()
  {
    while (FCGI_Accept() >= 0) {
      handle_request();
    }
    close_bb();
  }


  void handle_signal(int signum)
  {
    close_bb();
    FCGX_ShutdownPending();
  }

 private:
  void try_open_bb()
  {
    try {
      bb = new RemoteBlackBoard("localhost", 1910);
    } catch (Exception &e) {
      bb = NULL;
    }
  }


  void close_bb()
  {
    if ( bb != NULL ) {
      for (ifi = interfaces.begin(); ifi != interfaces.end(); ++ifi) {
	bb->close(ifi->second);
      }
      interfaces.clear();
      delete bb;
      bb = NULL;
    }
  }

  void handle_request()
  {
    if ( bb == NULL )  try_open_bb();

    std::string path_info = getenv("PATH_INFO");
    std::string script_name = getenv("SCRIPT_NAME");

    printf("Content-type: text/html\r\n"
	   "\r\n"
	   "<html>\n"
	   " <head>\n"
	   "  <title>Fawkes Webview</title>\n"
	   "  <link rel=\"stylesheet\" type=\"text/css\" href=\"/webview.css\" />\n"
	   " </head>\n"
	   " <body>\n"
	   "  <h1><img src=\"/webview.png\" /></h1>\n"
	   "  <p>Request number %d,  Process ID: %d, BlackBoard%s open</p>",
	   ++count, getpid(), (bb == NULL) ? " <i>not</i>" : "");

    
    //for (char **envp = environ; *envp != NULL; envp++) {
    //  printf("%s<br/>\n", *envp);
    //}

    if ( bb != NULL ) {
      printf("<ul>\n");
      InterfaceInfoList *iil = bb->list_all();
      for (InterfaceInfoList::iterator i = iil->begin(); i != iil->end(); ++i) {
	printf("<li>Interface <a href=\"%s/%s::%s\">%s::%s</a> (%u readers, has %s writer)</li>\n",
	       script_name.c_str(), i->type(), i->id(), i->type(), i->id(),
	       i->num_readers(), i->has_writer() ? "a" : "no");
      }

      if ( path_info != "" ) {
	std::string iuid = path_info.substr(path_info.find_first_not_of("/"));
	std::string iftype = iuid.substr(0, iuid.find("::"));
	std::string ifname = iuid.substr(iuid.find("::") + 2);

	printf("</ul>\n"
	       "<a href=\"%s\">Clear detailed</a>\n", script_name.c_str());

	printf("<h2>Interface: %s</h2>\n", iuid.c_str());
	if (interfaces.find(iuid) == interfaces.end()) {
	  try {
	    Interface *iface = bb->open_for_reading(iftype.c_str(), ifname.c_str());
	    interfaces[iuid] = iface;
	  } catch (Exception &e) {
	    printf("Failed to open interface: %s\n", e.what());
	  }
	}
	if (interfaces.find(iuid) != interfaces.end()) {
	  Interface *iface = interfaces[iuid];

	  printf("<table>\n"
		 " <tr><td><b>Type:</b></td><td>%s</td></tr>\n"
		 " <tr><td><b>ID:</b></td><td>%s</td></tr>\n"
		 " <tr><td><b>Has writer?:</b></td><td>%s</td></tr>\n"
		 " <tr><td><b>Num readers:</b></td><td>%u</td></tr>\n"
		 " <tr><td><b>Serial:</b></td><td>%u</td></tr>\n"
		 " <tr><td><b>Data size:</b></td><td>%u</td></tr>\n"
		 " <tr><td><b>Hash:</b></td><td>%s</td></tr>\n"
		 "</table>\n",
		 iface->type(), iface->id(), iface->has_writer() ? "yes" : "no",
		 iface->num_readers(), iface->serial(),
		 iface->datasize(), iface->hash_printable());


	  printf("<table>\n"
		 " <tr>\n"
		 "  <th>Name</th><th>Type</th><th>Value</th>\n"
		 " </tr>\n");
	  for (Interface::FieldIterator fi = iface->fields(); fi != iface->fields_end(); ++fi) {
	    printf(" <tr>\n"
		   "  <td>%s</td><td>%s</td><td>%s</td>\n"
		   " </tr>\n",
		   fi.get_name(), fi.get_typename(), fi.get_value_string());
	  }
	  printf("</table>\n");
	}
      }
    }
    printf(" </body>\n"
	   "</html>\n");
  }

 private:
  unsigned int  count;
  BlackBoard   *bb;
  std::map<std::string, Interface *> interfaces;
  std::map<std::string, Interface *>::iterator ifi;
};


/** Webview main.
 * @return 0
 */
int
main()
{
  FawkesWebviewMain wvm;

  SignalManager::register_handler(SIGINT, &wvm);
  SignalManager::register_handler(SIGUSR1, &wvm);

  wvm.run();

  return 0;
}
