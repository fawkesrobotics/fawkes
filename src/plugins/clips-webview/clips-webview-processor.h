
/***************************************************************************
 *  clips-webview-processor.h - CLIPS introspection via webview
 *
 *  Created: Sat Jun 15 20:15:18 2013
 *  Copyright  2006-2013  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_CLIPS_WEBVIEW_CLIPS_WEBVIEW_PROCESSOR_H_
#define __PLUGINS_CLIPS_WEBVIEW_CLIPS_WEBVIEW_PROCESSOR_H_

#include <core/utils/lockptr.h>

#include <string>
#include <list>

namespace fawkes {
  class Logger;
  class CLIPSEnvManager;
  class WebRequest;
  class WebReply;
}

namespace CLIPS {
  class Environment;
}

class ClipsWebRequestProcessor
{
 public:
  ClipsWebRequestProcessor(fawkes::LockPtr<fawkes::CLIPSEnvManager> &clips,
                           fawkes::Logger *logger);

  ~ClipsWebRequestProcessor();

  fawkes::WebReply *  process_assert(const fawkes::WebRequest *request);
  fawkes::WebReply *  process_retract(const fawkes::WebRequest *request);
  fawkes::WebReply *  process_environment(const fawkes::WebRequest *request);

  void add_error(const char *str);

 private:
  void retract_fact(fawkes::LockPtr<CLIPS::Environment> &clips, long int index);
  void enable_error_log(fawkes::LockPtr<CLIPS::Environment> &clips);
  void disable_error_log(fawkes::LockPtr<CLIPS::Environment> &clips);

 private:
  fawkes::LockPtr<fawkes::CLIPSEnvManager> clips_env_mgr_;
  fawkes::Logger       *logger_;

  std::list<std::string> errors_;
};

#endif
