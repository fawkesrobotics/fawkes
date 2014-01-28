
/***************************************************************************
 *  feature_redefine_warning.cpp -  CLIPS warning on redefinitions
 *
 *  Created: Tue Jan 21 20:31:17 2014
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#include "feature_redefine_warning.h"
#include <logging/logger.h>

extern "C" {
#include <clips/clips.h>
}
#include <clipsmm.h>

#define ROUTER_NAME "clips-feature-redefine-warn"

using namespace fawkes;


/// @cond INTERNALS
class CLIPSRedefineWarningLogger
{
 public:
  CLIPSRedefineWarningLogger(Logger *logger, const char *component = NULL)
  {
    logger_ = logger;
    if (component) {
      component_ = strdup(component);
    } else {
      component_ = NULL;
    }
    warn_filter_ = "[CSTRCPSR1] WARNING: ";
  }

  ~CLIPSRedefineWarningLogger()
  {
    if (component_) {
      free(component_);
    }
  }

  bool buffer_warning(const char *str, std::string &buffer_string)
  {
    if (strcmp(str, "\n") == 0) {
      if (warn_buffer_ == warn_filter_) {
	warn_buffer_.clear();
	return false;
      } else {
	buffer_string = warn_buffer_;
	warn_buffer_.clear();
	return true;
      }
    } else {
      warn_buffer_ += str;
      if (warn_filter_.find(warn_buffer_) == std::string::npos) {
	warn_buffer_.clear();
	buffer_string = str;
	return true;
      } else {
	return false;
      }
    }
  }

  void log(const char *str)
  {
    if (strcmp(str, "\n") == 0) {
      if (buffer_.find("Redefining ") == 0) {
	logger_->log_error(component_ ? component_ : "CLIPS", "%s", buffer_.c_str());
      }

      buffer_.clear();
    } else {
      buffer_ += str;
    }
  }

 private:
  Logger *logger_;
  char *component_;
  std::string buffer_;
  std::string warn_buffer_;
  std::string warn_filter_;
};

static int
redefine_warning_router_query(void *env, char *logical_name)
{
  if (strcmp(logical_name, WDIALOG) == 0) return TRUE;
  if (strcmp(logical_name, WWARNING) == 0) return TRUE;
  return FALSE;
}

static int
redefine_warning_router_print(void *env, char *logical_name, char *str)
{
  void *rc = GetEnvironmentRouterContext(env);
  CLIPSRedefineWarningLogger *logger = static_cast<CLIPSRedefineWarningLogger *>(rc);

  if (strcmp(logical_name, WWARNING) == 0) {
    // check if it's the ill-guided output of PrintWarningID from prntutil.c
    std::string wbuffer;
    ;
    if (logger->buffer_warning(str, wbuffer)) {
      // not the warning we were looking for, forward
      EnvDeactivateRouter(env, (char *)ROUTER_NAME);
      EnvPrintRouter(env, logical_name, (char *)wbuffer.c_str());
      if (strcmp(str, "\n") == 0 && wbuffer != "") {
	EnvPrintRouter(env, logical_name, str);
      }
      EnvActivateRouter(env, (char *)ROUTER_NAME);
    }
  } else {
    logger->log(str);
  }

  return TRUE;
}

static int
redefine_warning_router_exit(void *env, int exit_code)
{
  return TRUE;
}

/// @endcond


/** @class RedefineWarningCLIPSFeature "feature_redefine_warning.h"
 * CLIPS warning on redefinition of names.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param logger message logger
 */
RedefineWarningCLIPSFeature::RedefineWarningCLIPSFeature(fawkes::Logger *logger)
: CLIPSFeature("redefine-warning"), logger_(logger)
{
}


/** Destructor. */
RedefineWarningCLIPSFeature::~RedefineWarningCLIPSFeature()
{
}


void
RedefineWarningCLIPSFeature::clips_context_init(const std::string &env_name,
						fawkes::LockPtr<CLIPS::Environment> &clips)
{
  envs_[env_name] = clips;

  std::string name = "RWCLIPS|" + env_name;

  CLIPSRedefineWarningLogger *cl = new CLIPSRedefineWarningLogger(logger_, name.c_str());

  EnvAddRouterWithContext(clips->cobj(), (char *)ROUTER_NAME,
			  /* exclusive */ 40,
			  redefine_warning_router_query,
			  redefine_warning_router_print,
			  /* getc */   NULL,
			  /* ungetc */ NULL,
			  redefine_warning_router_exit,
			  cl);
  clips->watch("compilations");

}

void
RedefineWarningCLIPSFeature::clips_context_destroyed(const std::string &env_name)
{
  std::string name = "RWCLIPS|" + env_name;
  if (envs_.find(env_name) == envs_.end()) {
    logger_->log_warn(name.c_str(), "Environment %s has not been registered "
		      "for redefine warning feature", env_name.c_str());
    return;
  }

  fawkes::LockPtr<CLIPS::Environment> &clips = envs_[env_name];

  CLIPSRedefineWarningLogger *logger = NULL;

  struct routerData *rd = RouterData(clips->cobj());
  struct router *r = rd->ListOfRouters;
  while (r != NULL) {
    if (strcmp(r->name, ROUTER_NAME) == 0) {
      logger = static_cast<CLIPSRedefineWarningLogger *>(r->context);
      break;
    }
    r = r->next;
  }

  EnvDeleteRouter(clips->cobj(), (char *)ROUTER_NAME);
  delete logger;

  envs_.erase(env_name);
}
