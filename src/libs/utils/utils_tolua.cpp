/*
** Lua binding: utils
** Generated automatically by tolua++-1.0.92
*/

#ifndef __cplusplus
#include "stdlib.h"
#endif
#include "string.h"

#include "tolua++.h"

/* Exported function */
TOLUA_API int  tolua_utils_open (lua_State* tolua_S);

#include <utils/logging/component.h>
#include <utils/time/clock.h>
#include <utils/time/time.h>

/* function to release collected object via destructor */
#ifdef __cplusplus

static int tolua_collect_Time (lua_State* tolua_S)
{
 Time* self = (Time*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}
#endif


/* function to register type */
static void tolua_reg_types (lua_State* tolua_S)
{
 tolua_usertype(tolua_S,"Clock");
 tolua_usertype(tolua_S,"timeval");
 tolua_usertype(tolua_S,"ComponentLogger");
 tolua_usertype(tolua_S,"Time");
}

/* method: log_debug of class  ComponentLogger */
#ifndef TOLUA_DISABLE_tolua_utils_ComponentLogger_log_debug00
static int tolua_utils_ComponentLogger_log_debug00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ComponentLogger",0,&tolua_err) ||
     !tolua_iscppstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ComponentLogger* self = (ComponentLogger*)  tolua_tousertype(tolua_S,1,0);
  std::string message = ((std::string)  tolua_tocppstring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'log_debug'",NULL);
#endif
  {
   self->log_debug(message);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'log_debug'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: log_info of class  ComponentLogger */
#ifndef TOLUA_DISABLE_tolua_utils_ComponentLogger_log_info00
static int tolua_utils_ComponentLogger_log_info00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ComponentLogger",0,&tolua_err) ||
     !tolua_iscppstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ComponentLogger* self = (ComponentLogger*)  tolua_tousertype(tolua_S,1,0);
  std::string message = ((std::string)  tolua_tocppstring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'log_info'",NULL);
#endif
  {
   self->log_info(message);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'log_info'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: log_warn of class  ComponentLogger */
#ifndef TOLUA_DISABLE_tolua_utils_ComponentLogger_log_warn00
static int tolua_utils_ComponentLogger_log_warn00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ComponentLogger",0,&tolua_err) ||
     !tolua_iscppstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ComponentLogger* self = (ComponentLogger*)  tolua_tousertype(tolua_S,1,0);
  std::string message = ((std::string)  tolua_tocppstring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'log_warn'",NULL);
#endif
  {
   self->log_warn(message);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'log_warn'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: log_error of class  ComponentLogger */
#ifndef TOLUA_DISABLE_tolua_utils_ComponentLogger_log_error00
static int tolua_utils_ComponentLogger_log_error00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ComponentLogger",0,&tolua_err) ||
     !tolua_iscppstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ComponentLogger* self = (ComponentLogger*)  tolua_tousertype(tolua_S,1,0);
  std::string message = ((std::string)  tolua_tocppstring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'log_error'",NULL);
#endif
  {
   self->log_error(message);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'log_error'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: instance of class  Clock */
#ifndef TOLUA_DISABLE_tolua_utils_Clock_instance00
static int tolua_utils_Clock_instance00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"Clock",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   Clock* tolua_ret = (Clock*)  Clock::instance();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"Clock");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'instance'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: finalize of class  Clock */
#ifndef TOLUA_DISABLE_tolua_utils_Clock_finalize00
static int tolua_utils_Clock_finalize00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"Clock",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   Clock::finalize();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'finalize'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: is_ext_default_timesource of class  Clock */
#ifndef TOLUA_DISABLE_tolua_utils_Clock_is_ext_default_timesource00
static int tolua_utils_Clock_is_ext_default_timesource00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Clock",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Clock* self = (const Clock*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'is_ext_default_timesource'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->is_ext_default_timesource();
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'is_ext_default_timesource'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: has_ext_timesource of class  Clock */
#ifndef TOLUA_DISABLE_tolua_utils_Clock_has_ext_timesource00
static int tolua_utils_Clock_has_ext_timesource00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Clock",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Clock* self = (const Clock*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'has_ext_timesource'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->has_ext_timesource();
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'has_ext_timesource'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: ext_to_realtime of class  Clock */
#ifndef TOLUA_DISABLE_tolua_utils_Clock_ext_to_realtime00
static int tolua_utils_Clock_ext_to_realtime00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Clock",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"const Time",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Clock* self = (Clock*)  tolua_tousertype(tolua_S,1,0);
  const Time* t = ((const Time*)  tolua_tousertype(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'ext_to_realtime'",NULL);
#endif
  {
   Time tolua_ret = (Time)  self->ext_to_realtime(*t);
   {
#ifdef __cplusplus
    void* tolua_obj = new Time(tolua_ret);
    tolua_pushusertype_and_takeownership(tolua_S,tolua_obj,"Time");
#else
    void* tolua_obj = tolua_copy(tolua_S,(void*)&tolua_ret,sizeof(Time));
    tolua_pushusertype_and_takeownership(tolua_S,tolua_obj,"Time");
#endif
   }
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'ext_to_realtime'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: get_time of class  Clock */
#ifndef TOLUA_DISABLE_tolua_utils_Clock_get_time00
static int tolua_utils_Clock_get_time00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Clock",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"Time",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Clock* self = (const Clock*)  tolua_tousertype(tolua_S,1,0);
  Time* time = ((Time*)  tolua_tousertype(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'get_time'",NULL);
#endif
  {
   self->get_time(*time);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'get_time'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: get_time of class  Clock */
#ifndef TOLUA_DISABLE_tolua_utils_Clock_get_time01
static int tolua_utils_Clock_get_time01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Clock",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"Time",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  const Clock* self = (const Clock*)  tolua_tousertype(tolua_S,1,0);
  Time* time = ((Time*)  tolua_tousertype(tolua_S,2,0));
  Clock::TimesourceSelector sel = ((Clock::TimesourceSelector) (int)  tolua_tonumber(tolua_S,3,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'get_time'",NULL);
#endif
  {
   self->get_time(*time,sel);
  }
 }
 return 0;
tolua_lerror:
 return tolua_utils_Clock_get_time00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: get_time of class  Clock */
#ifndef TOLUA_DISABLE_tolua_utils_Clock_get_time02
static int tolua_utils_Clock_get_time02(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Clock",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"Time",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  const Clock* self = (const Clock*)  tolua_tousertype(tolua_S,1,0);
  Time* time = ((Time*)  tolua_tousertype(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'get_time'",NULL);
#endif
  {
   self->get_time(time);
  }
 }
 return 0;
tolua_lerror:
 return tolua_utils_Clock_get_time01(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: get_time of class  Clock */
#ifndef TOLUA_DISABLE_tolua_utils_Clock_get_time03
static int tolua_utils_Clock_get_time03(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Clock",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"Time",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  const Clock* self = (const Clock*)  tolua_tousertype(tolua_S,1,0);
  Time* time = ((Time*)  tolua_tousertype(tolua_S,2,0));
  Clock::TimesourceSelector sel = ((Clock::TimesourceSelector) (int)  tolua_tonumber(tolua_S,3,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'get_time'",NULL);
#endif
  {
   self->get_time(time,sel);
  }
 }
 return 0;
tolua_lerror:
 return tolua_utils_Clock_get_time02(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: get_time of class  Clock */
#ifndef TOLUA_DISABLE_tolua_utils_Clock_get_time04
static int tolua_utils_Clock_get_time04(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Clock",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"timeval",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  const Clock* self = (const Clock*)  tolua_tousertype(tolua_S,1,0);
  struct timeval* tv = ((struct timeval*)  tolua_tousertype(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'get_time'",NULL);
#endif
  {
   self->get_time(tv);
  }
 }
 return 0;
tolua_lerror:
 return tolua_utils_Clock_get_time03(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: get_time of class  Clock */
#ifndef TOLUA_DISABLE_tolua_utils_Clock_get_time05
static int tolua_utils_Clock_get_time05(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Clock",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"timeval",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  const Clock* self = (const Clock*)  tolua_tousertype(tolua_S,1,0);
  struct timeval* tv = ((struct timeval*)  tolua_tousertype(tolua_S,2,0));
  Clock::TimesourceSelector sel = ((Clock::TimesourceSelector) (int)  tolua_tonumber(tolua_S,3,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'get_time'",NULL);
#endif
  {
   self->get_time(tv,sel);
  }
 }
 return 0;
tolua_lerror:
 return tolua_utils_Clock_get_time04(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: get_systime of class  Clock */
#ifndef TOLUA_DISABLE_tolua_utils_Clock_get_systime00
static int tolua_utils_Clock_get_systime00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Clock",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"timeval",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Clock* self = (const Clock*)  tolua_tousertype(tolua_S,1,0);
  struct timeval* tv = ((struct timeval*)  tolua_tousertype(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'get_systime'",NULL);
#endif
  {
   self->get_systime(tv);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'get_systime'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: get_systime of class  Clock */
#ifndef TOLUA_DISABLE_tolua_utils_Clock_get_systime01
static int tolua_utils_Clock_get_systime01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Clock",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"Time",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  const Clock* self = (const Clock*)  tolua_tousertype(tolua_S,1,0);
  Time* time = ((Time*)  tolua_tousertype(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'get_systime'",NULL);
#endif
  {
   self->get_systime(*time);
  }
 }
 return 0;
tolua_lerror:
 return tolua_utils_Clock_get_systime00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: get_systime of class  Clock */
#ifndef TOLUA_DISABLE_tolua_utils_Clock_get_systime02
static int tolua_utils_Clock_get_systime02(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Clock",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"Time",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  const Clock* self = (const Clock*)  tolua_tousertype(tolua_S,1,0);
  Time* time = ((Time*)  tolua_tousertype(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'get_systime'",NULL);
#endif
  {
   self->get_systime(time);
  }
 }
 return 0;
tolua_lerror:
 return tolua_utils_Clock_get_systime01(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: now of class  Clock */
#ifndef TOLUA_DISABLE_tolua_utils_Clock_now00
static int tolua_utils_Clock_now00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Clock",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Clock* self = (const Clock*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'now'",NULL);
#endif
  {
   Time tolua_ret = (Time)  self->now();
   {
#ifdef __cplusplus
    void* tolua_obj = new Time(tolua_ret);
    tolua_pushusertype_and_takeownership(tolua_S,tolua_obj,"Time");
#else
    void* tolua_obj = tolua_copy(tolua_S,(void*)&tolua_ret,sizeof(Time));
    tolua_pushusertype_and_takeownership(tolua_S,tolua_obj,"Time");
#endif
   }
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'now'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: elapsed of class  Clock */
#ifndef TOLUA_DISABLE_tolua_utils_Clock_elapsed00
static int tolua_utils_Clock_elapsed00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Clock",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"Time",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Clock* self = (const Clock*)  tolua_tousertype(tolua_S,1,0);
  Time* t = ((Time*)  tolua_tousertype(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'elapsed'",NULL);
#endif
  {
   float tolua_ret = (float)  self->elapsed(t);
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'elapsed'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: sys_elapsed of class  Clock */
#ifndef TOLUA_DISABLE_tolua_utils_Clock_sys_elapsed00
static int tolua_utils_Clock_sys_elapsed00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Clock",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"Time",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Clock* self = (const Clock*)  tolua_tousertype(tolua_S,1,0);
  Time* t = ((Time*)  tolua_tousertype(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'sys_elapsed'",NULL);
#endif
  {
   float tolua_ret = (float)  self->sys_elapsed(t);
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'sys_elapsed'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* get function: tv_sec of class  timeval */
#ifndef TOLUA_DISABLE_tolua_get_timeval_tv_sec
static int tolua_get_timeval_tv_sec(lua_State* tolua_S)
{
  timeval* self = (timeval*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in accessing variable 'tv_sec'",NULL);
#endif
  tolua_pushnumber(tolua_S,(lua_Number)self->tv_sec);
 return 1;
}
#endif //#ifndef TOLUA_DISABLE

/* set function: tv_sec of class  timeval */
#ifndef TOLUA_DISABLE_tolua_set_timeval_tv_sec
static int tolua_set_timeval_tv_sec(lua_State* tolua_S)
{
  timeval* self = (timeval*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  tolua_Error tolua_err;
  if (!self) tolua_error(tolua_S,"invalid 'self' in accessing variable 'tv_sec'",NULL);
  if (!tolua_isnumber(tolua_S,2,0,&tolua_err))
   tolua_error(tolua_S,"#vinvalid type in variable assignment.",&tolua_err);
#endif
  self->tv_sec = ((long int)  tolua_tonumber(tolua_S,2,0))
;
 return 0;
}
#endif //#ifndef TOLUA_DISABLE

/* get function: tv_usec of class  timeval */
#ifndef TOLUA_DISABLE_tolua_get_timeval_tv_usec
static int tolua_get_timeval_tv_usec(lua_State* tolua_S)
{
  timeval* self = (timeval*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in accessing variable 'tv_usec'",NULL);
#endif
  tolua_pushnumber(tolua_S,(lua_Number)self->tv_usec);
 return 1;
}
#endif //#ifndef TOLUA_DISABLE

/* set function: tv_usec of class  timeval */
#ifndef TOLUA_DISABLE_tolua_set_timeval_tv_usec
static int tolua_set_timeval_tv_usec(lua_State* tolua_S)
{
  timeval* self = (timeval*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  tolua_Error tolua_err;
  if (!self) tolua_error(tolua_S,"invalid 'self' in accessing variable 'tv_usec'",NULL);
  if (!tolua_isnumber(tolua_S,2,0,&tolua_err))
   tolua_error(tolua_S,"#vinvalid type in variable assignment.",&tolua_err);
#endif
  self->tv_usec = ((long int)  tolua_tonumber(tolua_S,2,0))
;
 return 0;
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  Time */
#ifndef TOLUA_DISABLE_tolua_utils_Time_new00
static int tolua_utils_Time_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"Time",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   Time* tolua_ret = (Time*)  new Time();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"Time");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'new'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  Time */
#ifndef TOLUA_DISABLE_tolua_utils_Time_new00_local
static int tolua_utils_Time_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"Time",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   Time* tolua_ret = (Time*)  new Time();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"Time");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'new'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  Time */
#ifndef TOLUA_DISABLE_tolua_utils_Time_new01
static int tolua_utils_Time_new01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"Time",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  long ms = ((long)  tolua_tonumber(tolua_S,2,0));
  {
   Time* tolua_ret = (Time*)  new Time(ms);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"Time");
  }
 }
 return 1;
tolua_lerror:
 return tolua_utils_Time_new00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  Time */
#ifndef TOLUA_DISABLE_tolua_utils_Time_new01_local
static int tolua_utils_Time_new01_local(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"Time",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  long ms = ((long)  tolua_tonumber(tolua_S,2,0));
  {
   Time* tolua_ret = (Time*)  new Time(ms);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"Time");
  }
 }
 return 1;
tolua_lerror:
 return tolua_utils_Time_new00_local(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  Time */
#ifndef TOLUA_DISABLE_tolua_utils_Time_new02
static int tolua_utils_Time_new02(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"Time",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  float sec = ((float)  tolua_tonumber(tolua_S,2,0));
  {
   Time* tolua_ret = (Time*)  new Time(sec);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"Time");
  }
 }
 return 1;
tolua_lerror:
 return tolua_utils_Time_new01(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  Time */
#ifndef TOLUA_DISABLE_tolua_utils_Time_new02_local
static int tolua_utils_Time_new02_local(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"Time",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  float sec = ((float)  tolua_tonumber(tolua_S,2,0));
  {
   Time* tolua_ret = (Time*)  new Time(sec);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"Time");
  }
 }
 return 1;
tolua_lerror:
 return tolua_utils_Time_new01_local(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  Time */
#ifndef TOLUA_DISABLE_tolua_utils_Time_new03
static int tolua_utils_Time_new03(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"Time",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"Clock",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  Clock* clock = ((Clock*)  tolua_tousertype(tolua_S,2,0));
  {
   Time* tolua_ret = (Time*)  new Time(clock);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"Time");
  }
 }
 return 1;
tolua_lerror:
 return tolua_utils_Time_new02(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  Time */
#ifndef TOLUA_DISABLE_tolua_utils_Time_new03_local
static int tolua_utils_Time_new03_local(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"Time",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"Clock",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  Clock* clock = ((Clock*)  tolua_tousertype(tolua_S,2,0));
  {
   Time* tolua_ret = (Time*)  new Time(clock);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"Time");
  }
 }
 return 1;
tolua_lerror:
 return tolua_utils_Time_new02_local(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  Time */
#ifndef TOLUA_DISABLE_tolua_utils_Time_new04
static int tolua_utils_Time_new04(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"Time",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"const Time",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  const Time* t = ((const Time*)  tolua_tousertype(tolua_S,2,0));
  {
   Time* tolua_ret = (Time*)  new Time(*t);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"Time");
  }
 }
 return 1;
tolua_lerror:
 return tolua_utils_Time_new03(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  Time */
#ifndef TOLUA_DISABLE_tolua_utils_Time_new04_local
static int tolua_utils_Time_new04_local(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"Time",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"const Time",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  const Time* t = ((const Time*)  tolua_tousertype(tolua_S,2,0));
  {
   Time* tolua_ret = (Time*)  new Time(*t);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"Time");
  }
 }
 return 1;
tolua_lerror:
 return tolua_utils_Time_new03_local(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: delete of class  Time */
#ifndef TOLUA_DISABLE_tolua_utils_Time_delete00
static int tolua_utils_Time_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Time",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Time* self = (Time*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'delete'",NULL);
#endif
  delete self;
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'delete'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: in_sec of class  Time */
#ifndef TOLUA_DISABLE_tolua_utils_Time_in_sec00
static int tolua_utils_Time_in_sec00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Time",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Time* self = (const Time*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'in_sec'",NULL);
#endif
  {
   float tolua_ret = (float)  self->in_sec();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'in_sec'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: in_msec of class  Time */
#ifndef TOLUA_DISABLE_tolua_utils_Time_in_msec00
static int tolua_utils_Time_in_msec00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Time",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Time* self = (const Time*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'in_msec'",NULL);
#endif
  {
   long tolua_ret = (long)  self->in_msec();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'in_msec'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: in_usec of class  Time */
#ifndef TOLUA_DISABLE_tolua_utils_Time_in_usec00
static int tolua_utils_Time_in_usec00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Time",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Time* self = (const Time*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'in_usec'",NULL);
#endif
  {
   long tolua_ret = (long)  self->in_usec();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'in_usec'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: get_timeval of class  Time */
#ifndef TOLUA_DISABLE_tolua_utils_Time_get_timeval00
static int tolua_utils_Time_get_timeval00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Time",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Time* self = (const Time*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'get_timeval'",NULL);
#endif
  {
   const timeval* tolua_ret = (const timeval*)  self->get_timeval();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"const timeval");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'get_timeval'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_time of class  Time */
#ifndef TOLUA_DISABLE_tolua_utils_Time_set_time00
static int tolua_utils_Time_set_time00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Time",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Time* self = (Time*)  tolua_tousertype(tolua_S,1,0);
  long ms = ((long)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_time'",NULL);
#endif
  {
   self->set_time(ms);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_time'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_time of class  Time */
#ifndef TOLUA_DISABLE_tolua_utils_Time_set_time01
static int tolua_utils_Time_set_time01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Time",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  Time* self = (Time*)  tolua_tousertype(tolua_S,1,0);
  float sec = ((float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_time'",NULL);
#endif
  {
   self->set_time(sec);
  }
 }
 return 0;
tolua_lerror:
 return tolua_utils_Time_set_time00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: stamp of class  Time */
#ifndef TOLUA_DISABLE_tolua_utils_Time_stamp00
static int tolua_utils_Time_stamp00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Time",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Time* self = (Time*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'stamp'",NULL);
#endif
  {
   Time& tolua_ret = (Time&)  self->stamp();
   tolua_pushusertype(tolua_S,(void*)&tolua_ret,"Time");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'stamp'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: operator+ of class  Time */
#ifndef TOLUA_DISABLE_tolua_utils_Time__add00
static int tolua_utils_Time__add00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Time",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Time* self = (const Time*)  tolua_tousertype(tolua_S,1,0);
  const float sec = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'operator+'",NULL);
#endif
  {
   Time tolua_ret = (Time)  self->operator+(sec);
   {
#ifdef __cplusplus
    void* tolua_obj = new Time(tolua_ret);
    tolua_pushusertype_and_takeownership(tolua_S,tolua_obj,"Time");
#else
    void* tolua_obj = tolua_copy(tolua_S,(void*)&tolua_ret,sizeof(Time));
    tolua_pushusertype_and_takeownership(tolua_S,tolua_obj,"Time");
#endif
   }
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function '.add'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: operator+ of class  Time */
#ifndef TOLUA_DISABLE_tolua_utils_Time__add01
static int tolua_utils_Time__add01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Time",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"const Time",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  const Time* self = (const Time*)  tolua_tousertype(tolua_S,1,0);
  const Time* t = ((const Time*)  tolua_tousertype(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'operator+'",NULL);
#endif
  {
   Time tolua_ret = (Time)  self->operator+(*t);
   {
#ifdef __cplusplus
    void* tolua_obj = new Time(tolua_ret);
    tolua_pushusertype_and_takeownership(tolua_S,tolua_obj,"Time");
#else
    void* tolua_obj = tolua_copy(tolua_S,(void*)&tolua_ret,sizeof(Time));
    tolua_pushusertype_and_takeownership(tolua_S,tolua_obj,"Time");
#endif
   }
  }
 }
 return 1;
tolua_lerror:
 return tolua_utils_Time__add00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: operator- of class  Time */
#ifndef TOLUA_DISABLE_tolua_utils_Time__sub00
static int tolua_utils_Time__sub00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Time",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"const Time",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Time* self = (const Time*)  tolua_tousertype(tolua_S,1,0);
  const Time* t = ((const Time*)  tolua_tousertype(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'operator-'",NULL);
#endif
  {
   Time tolua_ret = (Time)  self->operator-(*t);
   {
#ifdef __cplusplus
    void* tolua_obj = new Time(tolua_ret);
    tolua_pushusertype_and_takeownership(tolua_S,tolua_obj,"Time");
#else
    void* tolua_obj = tolua_copy(tolua_S,(void*)&tolua_ret,sizeof(Time));
    tolua_pushusertype_and_takeownership(tolua_S,tolua_obj,"Time");
#endif
   }
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function '.sub'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: operator- of class  Time */
#ifndef TOLUA_DISABLE_tolua_utils_Time__sub01
static int tolua_utils_Time__sub01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Time",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"const Time",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  const Time* self = (const Time*)  tolua_tousertype(tolua_S,1,0);
  const Time* t = ((const Time*)  tolua_tousertype(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'operator-'",NULL);
#endif
  {
   float tolua_ret = (float)  self->operator-(t);
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
tolua_lerror:
 return tolua_utils_Time__sub00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: str of class  Time */
#ifndef TOLUA_DISABLE_tolua_utils_Time_str00
static int tolua_utils_Time_str00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Time",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Time* self = (Time*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'str'",NULL);
#endif
  {
   const char* tolua_ret = (const char*)  self->str();
   tolua_pushstring(tolua_S,(const char*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'str'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: str_r of class  Time */
#ifndef TOLUA_DISABLE_tolua_utils_Time_str_r00
static int tolua_utils_Time_str_r00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Time",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Time* self = (Time*)  tolua_tousertype(tolua_S,1,0);
  char* s = ((char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'str_r'",NULL);
#endif
  {
   self->str_r(s);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'str_r'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* Open function */
TOLUA_API int tolua_utils_open (lua_State* tolua_S)
{
 tolua_open(tolua_S);
 tolua_reg_types(tolua_S);
 tolua_module(tolua_S,NULL,0);
 tolua_beginmodule(tolua_S,NULL);
  tolua_cclass(tolua_S,"ComponentLogger","ComponentLogger","",NULL);
  tolua_beginmodule(tolua_S,"ComponentLogger");
   tolua_function(tolua_S,"log_debug",tolua_utils_ComponentLogger_log_debug00);
   tolua_function(tolua_S,"log_info",tolua_utils_ComponentLogger_log_info00);
   tolua_function(tolua_S,"log_warn",tolua_utils_ComponentLogger_log_warn00);
   tolua_function(tolua_S,"log_error",tolua_utils_ComponentLogger_log_error00);
  tolua_endmodule(tolua_S);
  tolua_cclass(tolua_S,"Clock","Clock","",NULL);
  tolua_beginmodule(tolua_S,"Clock");
   tolua_constant(tolua_S,"DEFAULT",Clock::DEFAULT);
   tolua_constant(tolua_S,"REALTIME",Clock::REALTIME);
   tolua_constant(tolua_S,"EXTERNAL",Clock::EXTERNAL);
   tolua_function(tolua_S,"instance",tolua_utils_Clock_instance00);
   tolua_function(tolua_S,"finalize",tolua_utils_Clock_finalize00);
   tolua_function(tolua_S,"is_ext_default_timesource",tolua_utils_Clock_is_ext_default_timesource00);
   tolua_function(tolua_S,"has_ext_timesource",tolua_utils_Clock_has_ext_timesource00);
   tolua_function(tolua_S,"ext_to_realtime",tolua_utils_Clock_ext_to_realtime00);
   tolua_function(tolua_S,"get_time",tolua_utils_Clock_get_time00);
   tolua_function(tolua_S,"get_time",tolua_utils_Clock_get_time01);
   tolua_function(tolua_S,"get_time",tolua_utils_Clock_get_time02);
   tolua_function(tolua_S,"get_time",tolua_utils_Clock_get_time03);
   tolua_function(tolua_S,"get_time",tolua_utils_Clock_get_time04);
   tolua_function(tolua_S,"get_time",tolua_utils_Clock_get_time05);
   tolua_function(tolua_S,"get_systime",tolua_utils_Clock_get_systime00);
   tolua_function(tolua_S,"get_systime",tolua_utils_Clock_get_systime01);
   tolua_function(tolua_S,"get_systime",tolua_utils_Clock_get_systime02);
   tolua_function(tolua_S,"now",tolua_utils_Clock_now00);
   tolua_function(tolua_S,"elapsed",tolua_utils_Clock_elapsed00);
   tolua_function(tolua_S,"sys_elapsed",tolua_utils_Clock_sys_elapsed00);
  tolua_endmodule(tolua_S);
  tolua_cclass(tolua_S,"timeval","timeval","",NULL);
  tolua_beginmodule(tolua_S,"timeval");
   tolua_variable(tolua_S,"tv_sec",tolua_get_timeval_tv_sec,tolua_set_timeval_tv_sec);
   tolua_variable(tolua_S,"tv_usec",tolua_get_timeval_tv_usec,tolua_set_timeval_tv_usec);
  tolua_endmodule(tolua_S);
  #ifdef __cplusplus
  tolua_cclass(tolua_S,"Time","Time","",tolua_collect_Time);
  #else
  tolua_cclass(tolua_S,"Time","Time","",NULL);
  #endif
  tolua_beginmodule(tolua_S,"Time");
   tolua_function(tolua_S,"new",tolua_utils_Time_new00);
   tolua_function(tolua_S,"new_local",tolua_utils_Time_new00_local);
   tolua_function(tolua_S,".call",tolua_utils_Time_new00_local);
   tolua_function(tolua_S,"new",tolua_utils_Time_new01);
   tolua_function(tolua_S,"new_local",tolua_utils_Time_new01_local);
   tolua_function(tolua_S,".call",tolua_utils_Time_new01_local);
   tolua_function(tolua_S,"new",tolua_utils_Time_new02);
   tolua_function(tolua_S,"new_local",tolua_utils_Time_new02_local);
   tolua_function(tolua_S,".call",tolua_utils_Time_new02_local);
   tolua_function(tolua_S,"new",tolua_utils_Time_new03);
   tolua_function(tolua_S,"new_local",tolua_utils_Time_new03_local);
   tolua_function(tolua_S,".call",tolua_utils_Time_new03_local);
   tolua_function(tolua_S,"new",tolua_utils_Time_new04);
   tolua_function(tolua_S,"new_local",tolua_utils_Time_new04_local);
   tolua_function(tolua_S,".call",tolua_utils_Time_new04_local);
   tolua_function(tolua_S,"delete",tolua_utils_Time_delete00);
   tolua_function(tolua_S,"in_sec",tolua_utils_Time_in_sec00);
   tolua_function(tolua_S,"in_msec",tolua_utils_Time_in_msec00);
   tolua_function(tolua_S,"in_usec",tolua_utils_Time_in_usec00);
   tolua_function(tolua_S,"get_timeval",tolua_utils_Time_get_timeval00);
   tolua_function(tolua_S,"set_time",tolua_utils_Time_set_time00);
   tolua_function(tolua_S,"set_time",tolua_utils_Time_set_time01);
   tolua_function(tolua_S,"stamp",tolua_utils_Time_stamp00);
   tolua_function(tolua_S,".add",tolua_utils_Time__add00);
   tolua_function(tolua_S,".add",tolua_utils_Time__add01);
   tolua_function(tolua_S,".sub",tolua_utils_Time__sub00);
   tolua_function(tolua_S,".sub",tolua_utils_Time__sub01);
   tolua_function(tolua_S,"str",tolua_utils_Time_str00);
   tolua_function(tolua_S,"str_r",tolua_utils_Time_str_r00);
  tolua_endmodule(tolua_S);
 tolua_endmodule(tolua_S);
 return 1;
}


extern "C" {
#if defined(LUA_VERSION_NUM) && LUA_VERSION_NUM >= 501
 TOLUA_API int luaopen_utils (lua_State* tolua_S) {
 return tolua_utils_open(tolua_S);
};
#endif
}


