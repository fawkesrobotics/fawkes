/*
** Lua binding: interfaces
** Generated automatically by tolua++-1.0.92
*/

#ifndef __cplusplus
#include "stdlib.h"
#endif
#include "string.h"

#include "tolua++.h"

/* Exported function */
TOLUA_API int  tolua_interfaces_open (lua_State* tolua_S);

#include <interfaces/battery.h>
#include <interfaces/kicker.h>
#include <interfaces/motor.h>
#include <interfaces/navigator.h>
#include <interfaces/object.h>
#include <interfaces/skiller.h>
#include <interfaces/test.h>

/* function to release collected object via destructor */
#ifdef __cplusplus

static int tolua_collect_NavigatorInterface__ObstacleMessage (lua_State* tolua_S)
{
 NavigatorInterface::ObstacleMessage* self = (NavigatorInterface::ObstacleMessage*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_TestInterface__CalculateMessage (lua_State* tolua_S)
{
 TestInterface::CalculateMessage* self = (TestInterface::CalculateMessage*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_MotorInterface__RotMessage (lua_State* tolua_S)
{
 MotorInterface::RotMessage* self = (MotorInterface::RotMessage*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_KickerInterface__KickMessage (lua_State* tolua_S)
{
 KickerInterface::KickMessage* self = (KickerInterface::KickMessage*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_SkillerInterface__CallSkillMessage (lua_State* tolua_S)
{
 SkillerInterface::CallSkillMessage* self = (SkillerInterface::CallSkillMessage*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_KickerInterface__GuideBallMessage (lua_State* tolua_S)
{
 KickerInterface::GuideBallMessage* self = (KickerInterface::GuideBallMessage*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_BatteryInterface__PushButtonMessage (lua_State* tolua_S)
{
 BatteryInterface::PushButtonMessage* self = (BatteryInterface::PushButtonMessage*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_MotorInterface__SetMotorStateMessage (lua_State* tolua_S)
{
 MotorInterface::SetMotorStateMessage* self = (MotorInterface::SetMotorStateMessage*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_TestInterface__SetTestStringMessage (lua_State* tolua_S)
{
 TestInterface::SetTestStringMessage* self = (TestInterface::SetTestStringMessage*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_MotorInterface__TransMessage (lua_State* tolua_S)
{
 MotorInterface::TransMessage* self = (MotorInterface::TransMessage*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_BatteryInterface__SleepMessage (lua_State* tolua_S)
{
 BatteryInterface::SleepMessage* self = (BatteryInterface::SleepMessage*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_MotorInterface__AcquireControlMessage (lua_State* tolua_S)
{
 MotorInterface::AcquireControlMessage* self = (MotorInterface::AcquireControlMessage*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_MotorInterface__OrbitMessage (lua_State* tolua_S)
{
 MotorInterface::OrbitMessage* self = (MotorInterface::OrbitMessage*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_TestInterface__SetTestIntMessage (lua_State* tolua_S)
{
 TestInterface::SetTestIntMessage* self = (TestInterface::SetTestIntMessage*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_SkillerInterface__RestartInterpreterMessage (lua_State* tolua_S)
{
 SkillerInterface::RestartInterpreterMessage* self = (SkillerInterface::RestartInterpreterMessage*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_MotorInterface__TransRotMessage (lua_State* tolua_S)
{
 MotorInterface::TransRotMessage* self = (MotorInterface::TransRotMessage*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_NavigatorInterface__MaxVelocityMessage (lua_State* tolua_S)
{
 NavigatorInterface::MaxVelocityMessage* self = (NavigatorInterface::MaxVelocityMessage*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_NavigatorInterface__TargetMessage (lua_State* tolua_S)
{
 NavigatorInterface::TargetMessage* self = (NavigatorInterface::TargetMessage*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_MotorInterface__DriveRPMMessage (lua_State* tolua_S)
{
 MotorInterface::DriveRPMMessage* self = (MotorInterface::DriveRPMMessage*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_MotorInterface__ResetOdometryMessage (lua_State* tolua_S)
{
 MotorInterface::ResetOdometryMessage* self = (MotorInterface::ResetOdometryMessage*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_KickerInterface__ResetCounterMessage (lua_State* tolua_S)
{
 KickerInterface::ResetCounterMessage* self = (KickerInterface::ResetCounterMessage*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_MotorInterface__LinTransRotMessage (lua_State* tolua_S)
{
 MotorInterface::LinTransRotMessage* self = (MotorInterface::LinTransRotMessage*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}
#endif


/* function to register type */
static void tolua_reg_types (lua_State* tolua_S)
{
 tolua_usertype(tolua_S,"NavigatorInterface::ObstacleMessage");
 tolua_usertype(tolua_S,"TestInterface::CalculateMessage");
 tolua_usertype(tolua_S,"TestInterface::SetTestStringMessage");
 tolua_usertype(tolua_S,"ObjectPositionInterface");
 tolua_usertype(tolua_S,"TestInterface::SetTestIntMessage");
 tolua_usertype(tolua_S,"KickerInterface");
 tolua_usertype(tolua_S,"SkillerInterface::RestartInterpreterMessage");
 tolua_usertype(tolua_S,"MotorInterface::RotMessage");
 tolua_usertype(tolua_S,"KickerInterface::KickMessage");
 tolua_usertype(tolua_S,"SkillerInterface::CallSkillMessage");
 tolua_usertype(tolua_S,"TestInterface");
 tolua_usertype(tolua_S,"KickerInterface::GuideBallMessage");
 tolua_usertype(tolua_S,"BatteryInterface::PushButtonMessage");
 tolua_usertype(tolua_S,"MotorInterface::SetMotorStateMessage");
 tolua_usertype(tolua_S,"Interface");
 tolua_usertype(tolua_S,"SkillerInterface");
 tolua_usertype(tolua_S,"NavigatorInterface::MaxVelocityMessage");
 tolua_usertype(tolua_S,"NavigatorInterface::TargetMessage");
 tolua_usertype(tolua_S,"MotorInterface::TransMessage");
 tolua_usertype(tolua_S,"BatteryInterface::SleepMessage");
 tolua_usertype(tolua_S,"MotorInterface::AcquireControlMessage");
 tolua_usertype(tolua_S,"MotorInterface::OrbitMessage");
 tolua_usertype(tolua_S,"NavigatorInterface");
 tolua_usertype(tolua_S,"Message");
 tolua_usertype(tolua_S,"MotorInterface::TransRotMessage");
 tolua_usertype(tolua_S,"BatteryInterface");
 tolua_usertype(tolua_S,"MotorInterface");
 tolua_usertype(tolua_S,"MotorInterface::DriveRPMMessage");
 tolua_usertype(tolua_S,"MotorInterface::ResetOdometryMessage");
 tolua_usertype(tolua_S,"KickerInterface::ResetCounterMessage");
 tolua_usertype(tolua_S,"MotorInterface::LinTransRotMessage");
}

/* method: new of class  PushButtonMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_BatteryInterface_PushButtonMessage_new00
static int tolua_interfaces_BatteryInterface_PushButtonMessage_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"BatteryInterface::PushButtonMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   BatteryInterface::PushButtonMessage* tolua_ret = (BatteryInterface::PushButtonMessage*)  new BatteryInterface::PushButtonMessage();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"BatteryInterface::PushButtonMessage");
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

/* method: new_local of class  PushButtonMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_BatteryInterface_PushButtonMessage_new00_local
static int tolua_interfaces_BatteryInterface_PushButtonMessage_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"BatteryInterface::PushButtonMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   BatteryInterface::PushButtonMessage* tolua_ret = (BatteryInterface::PushButtonMessage*)  new BatteryInterface::PushButtonMessage();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"BatteryInterface::PushButtonMessage");
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

/* method: delete of class  PushButtonMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_BatteryInterface_PushButtonMessage_delete00
static int tolua_interfaces_BatteryInterface_PushButtonMessage_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"BatteryInterface::PushButtonMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  BatteryInterface::PushButtonMessage* self = (BatteryInterface::PushButtonMessage*)  tolua_tousertype(tolua_S,1,0);
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

/* method: new of class  SleepMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_BatteryInterface_SleepMessage_new00
static int tolua_interfaces_BatteryInterface_SleepMessage_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"BatteryInterface::SleepMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   BatteryInterface::SleepMessage* tolua_ret = (BatteryInterface::SleepMessage*)  new BatteryInterface::SleepMessage();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"BatteryInterface::SleepMessage");
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

/* method: new_local of class  SleepMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_BatteryInterface_SleepMessage_new00_local
static int tolua_interfaces_BatteryInterface_SleepMessage_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"BatteryInterface::SleepMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   BatteryInterface::SleepMessage* tolua_ret = (BatteryInterface::SleepMessage*)  new BatteryInterface::SleepMessage();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"BatteryInterface::SleepMessage");
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

/* method: delete of class  SleepMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_BatteryInterface_SleepMessage_delete00
static int tolua_interfaces_BatteryInterface_SleepMessage_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"BatteryInterface::SleepMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  BatteryInterface::SleepMessage* self = (BatteryInterface::SleepMessage*)  tolua_tousertype(tolua_S,1,0);
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

/* method: current of class  BatteryInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_BatteryInterface_current00
static int tolua_interfaces_BatteryInterface_current00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"BatteryInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  BatteryInterface* self = (BatteryInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'current'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->current();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'current'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_current of class  BatteryInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_BatteryInterface_set_current00
static int tolua_interfaces_BatteryInterface_set_current00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"BatteryInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  BatteryInterface* self = (BatteryInterface*)  tolua_tousertype(tolua_S,1,0);
  unsigned const int new_current = ((unsigned const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_current'",NULL);
#endif
  {
   self->set_current(new_current);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_current'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: voltage of class  BatteryInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_BatteryInterface_voltage00
static int tolua_interfaces_BatteryInterface_voltage00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"BatteryInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  BatteryInterface* self = (BatteryInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'voltage'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->voltage();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'voltage'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_voltage of class  BatteryInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_BatteryInterface_set_voltage00
static int tolua_interfaces_BatteryInterface_set_voltage00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"BatteryInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  BatteryInterface* self = (BatteryInterface*)  tolua_tousertype(tolua_S,1,0);
  unsigned const int new_voltage = ((unsigned const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_voltage'",NULL);
#endif
  {
   self->set_voltage(new_voltage);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_voltage'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: temperature of class  BatteryInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_BatteryInterface_temperature00
static int tolua_interfaces_BatteryInterface_temperature00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"BatteryInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  BatteryInterface* self = (BatteryInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'temperature'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->temperature();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'temperature'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_temperature of class  BatteryInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_BatteryInterface_set_temperature00
static int tolua_interfaces_BatteryInterface_set_temperature00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"BatteryInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  BatteryInterface* self = (BatteryInterface*)  tolua_tousertype(tolua_S,1,0);
  unsigned const int new_temperature = ((unsigned const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_temperature'",NULL);
#endif
  {
   self->set_temperature(new_temperature);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_temperature'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  KickMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_KickMessage_new00
static int tolua_interfaces_KickerInterface_KickMessage_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"KickerInterface::KickMessage",0,&tolua_err) ||
     !tolua_isboolean(tolua_S,2,0,&tolua_err) ||
     !tolua_isboolean(tolua_S,3,0,&tolua_err) ||
     !tolua_isboolean(tolua_S,4,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,5,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,6,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  bool ini_left = ((bool)  tolua_toboolean(tolua_S,2,0));
  bool ini_center = ((bool)  tolua_toboolean(tolua_S,3,0));
  bool ini_right = ((bool)  tolua_toboolean(tolua_S,4,0));
  unsigned int ini_intensity = ((unsigned int)  tolua_tonumber(tolua_S,5,0));
  {
   KickerInterface::KickMessage* tolua_ret = (KickerInterface::KickMessage*)  new KickerInterface::KickMessage(ini_left,ini_center,ini_right,ini_intensity);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"KickerInterface::KickMessage");
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

/* method: new_local of class  KickMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_KickMessage_new00_local
static int tolua_interfaces_KickerInterface_KickMessage_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"KickerInterface::KickMessage",0,&tolua_err) ||
     !tolua_isboolean(tolua_S,2,0,&tolua_err) ||
     !tolua_isboolean(tolua_S,3,0,&tolua_err) ||
     !tolua_isboolean(tolua_S,4,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,5,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,6,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  bool ini_left = ((bool)  tolua_toboolean(tolua_S,2,0));
  bool ini_center = ((bool)  tolua_toboolean(tolua_S,3,0));
  bool ini_right = ((bool)  tolua_toboolean(tolua_S,4,0));
  unsigned int ini_intensity = ((unsigned int)  tolua_tonumber(tolua_S,5,0));
  {
   KickerInterface::KickMessage* tolua_ret = (KickerInterface::KickMessage*)  new KickerInterface::KickMessage(ini_left,ini_center,ini_right,ini_intensity);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"KickerInterface::KickMessage");
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

/* method: new of class  KickMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_KickMessage_new01
static int tolua_interfaces_KickerInterface_KickMessage_new01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"KickerInterface::KickMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   KickerInterface::KickMessage* tolua_ret = (KickerInterface::KickMessage*)  new KickerInterface::KickMessage();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"KickerInterface::KickMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_KickerInterface_KickMessage_new00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  KickMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_KickMessage_new01_local
static int tolua_interfaces_KickerInterface_KickMessage_new01_local(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"KickerInterface::KickMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   KickerInterface::KickMessage* tolua_ret = (KickerInterface::KickMessage*)  new KickerInterface::KickMessage();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"KickerInterface::KickMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_KickerInterface_KickMessage_new00_local(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: delete of class  KickMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_KickMessage_delete00
static int tolua_interfaces_KickerInterface_KickMessage_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"KickerInterface::KickMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface::KickMessage* self = (KickerInterface::KickMessage*)  tolua_tousertype(tolua_S,1,0);
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

/* method: is_left of class  KickMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_KickMessage_is_left00
static int tolua_interfaces_KickerInterface_KickMessage_is_left00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"KickerInterface::KickMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface::KickMessage* self = (KickerInterface::KickMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'is_left'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->is_left();
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'is_left'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_left of class  KickMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_KickMessage_set_left00
static int tolua_interfaces_KickerInterface_KickMessage_set_left00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"KickerInterface::KickMessage",0,&tolua_err) ||
     !tolua_isboolean(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface::KickMessage* self = (KickerInterface::KickMessage*)  tolua_tousertype(tolua_S,1,0);
  const bool new_left = ((const bool)  tolua_toboolean(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_left'",NULL);
#endif
  {
   self->set_left(new_left);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_left'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: is_center of class  KickMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_KickMessage_is_center00
static int tolua_interfaces_KickerInterface_KickMessage_is_center00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"KickerInterface::KickMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface::KickMessage* self = (KickerInterface::KickMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'is_center'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->is_center();
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'is_center'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_center of class  KickMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_KickMessage_set_center00
static int tolua_interfaces_KickerInterface_KickMessage_set_center00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"KickerInterface::KickMessage",0,&tolua_err) ||
     !tolua_isboolean(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface::KickMessage* self = (KickerInterface::KickMessage*)  tolua_tousertype(tolua_S,1,0);
  const bool new_center = ((const bool)  tolua_toboolean(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_center'",NULL);
#endif
  {
   self->set_center(new_center);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_center'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: is_right of class  KickMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_KickMessage_is_right00
static int tolua_interfaces_KickerInterface_KickMessage_is_right00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"KickerInterface::KickMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface::KickMessage* self = (KickerInterface::KickMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'is_right'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->is_right();
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'is_right'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_right of class  KickMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_KickMessage_set_right00
static int tolua_interfaces_KickerInterface_KickMessage_set_right00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"KickerInterface::KickMessage",0,&tolua_err) ||
     !tolua_isboolean(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface::KickMessage* self = (KickerInterface::KickMessage*)  tolua_tousertype(tolua_S,1,0);
  const bool new_right = ((const bool)  tolua_toboolean(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_right'",NULL);
#endif
  {
   self->set_right(new_right);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_right'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: intensity of class  KickMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_KickMessage_intensity00
static int tolua_interfaces_KickerInterface_KickMessage_intensity00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"KickerInterface::KickMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface::KickMessage* self = (KickerInterface::KickMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'intensity'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->intensity();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'intensity'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_intensity of class  KickMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_KickMessage_set_intensity00
static int tolua_interfaces_KickerInterface_KickMessage_set_intensity00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"KickerInterface::KickMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface::KickMessage* self = (KickerInterface::KickMessage*)  tolua_tousertype(tolua_S,1,0);
  unsigned const int new_intensity = ((unsigned const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_intensity'",NULL);
#endif
  {
   self->set_intensity(new_intensity);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_intensity'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  ResetCounterMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_ResetCounterMessage_new00
static int tolua_interfaces_KickerInterface_ResetCounterMessage_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"KickerInterface::ResetCounterMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   KickerInterface::ResetCounterMessage* tolua_ret = (KickerInterface::ResetCounterMessage*)  new KickerInterface::ResetCounterMessage();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"KickerInterface::ResetCounterMessage");
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

/* method: new_local of class  ResetCounterMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_ResetCounterMessage_new00_local
static int tolua_interfaces_KickerInterface_ResetCounterMessage_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"KickerInterface::ResetCounterMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   KickerInterface::ResetCounterMessage* tolua_ret = (KickerInterface::ResetCounterMessage*)  new KickerInterface::ResetCounterMessage();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"KickerInterface::ResetCounterMessage");
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

/* method: delete of class  ResetCounterMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_ResetCounterMessage_delete00
static int tolua_interfaces_KickerInterface_ResetCounterMessage_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"KickerInterface::ResetCounterMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface::ResetCounterMessage* self = (KickerInterface::ResetCounterMessage*)  tolua_tousertype(tolua_S,1,0);
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

/* method: new of class  GuideBallMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_GuideBallMessage_new00
static int tolua_interfaces_KickerInterface_GuideBallMessage_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"KickerInterface::GuideBallMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface::GuideBallSideEnum ini_guide_ball_side = ((KickerInterface::GuideBallSideEnum) (int)  tolua_tonumber(tolua_S,2,0));
  {
   KickerInterface::GuideBallMessage* tolua_ret = (KickerInterface::GuideBallMessage*)  new KickerInterface::GuideBallMessage(ini_guide_ball_side);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"KickerInterface::GuideBallMessage");
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

/* method: new_local of class  GuideBallMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_GuideBallMessage_new00_local
static int tolua_interfaces_KickerInterface_GuideBallMessage_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"KickerInterface::GuideBallMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface::GuideBallSideEnum ini_guide_ball_side = ((KickerInterface::GuideBallSideEnum) (int)  tolua_tonumber(tolua_S,2,0));
  {
   KickerInterface::GuideBallMessage* tolua_ret = (KickerInterface::GuideBallMessage*)  new KickerInterface::GuideBallMessage(ini_guide_ball_side);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"KickerInterface::GuideBallMessage");
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

/* method: new of class  GuideBallMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_GuideBallMessage_new01
static int tolua_interfaces_KickerInterface_GuideBallMessage_new01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"KickerInterface::GuideBallMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   KickerInterface::GuideBallMessage* tolua_ret = (KickerInterface::GuideBallMessage*)  new KickerInterface::GuideBallMessage();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"KickerInterface::GuideBallMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_KickerInterface_GuideBallMessage_new00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  GuideBallMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_GuideBallMessage_new01_local
static int tolua_interfaces_KickerInterface_GuideBallMessage_new01_local(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"KickerInterface::GuideBallMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   KickerInterface::GuideBallMessage* tolua_ret = (KickerInterface::GuideBallMessage*)  new KickerInterface::GuideBallMessage();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"KickerInterface::GuideBallMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_KickerInterface_GuideBallMessage_new00_local(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: delete of class  GuideBallMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_GuideBallMessage_delete00
static int tolua_interfaces_KickerInterface_GuideBallMessage_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"KickerInterface::GuideBallMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface::GuideBallMessage* self = (KickerInterface::GuideBallMessage*)  tolua_tousertype(tolua_S,1,0);
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

/* method: guide_ball_side of class  GuideBallMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_GuideBallMessage_guide_ball_side00
static int tolua_interfaces_KickerInterface_GuideBallMessage_guide_ball_side00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"KickerInterface::GuideBallMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface::GuideBallMessage* self = (KickerInterface::GuideBallMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'guide_ball_side'",NULL);
#endif
  {
   KickerInterface::GuideBallSideEnum tolua_ret = (KickerInterface::GuideBallSideEnum)  self->guide_ball_side();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'guide_ball_side'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_guide_ball_side of class  GuideBallMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_GuideBallMessage_set_guide_ball_side00
static int tolua_interfaces_KickerInterface_GuideBallMessage_set_guide_ball_side00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"KickerInterface::GuideBallMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface::GuideBallMessage* self = (KickerInterface::GuideBallMessage*)  tolua_tousertype(tolua_S,1,0);
  const KickerInterface::GuideBallSideEnum new_guide_ball_side = ((const KickerInterface::GuideBallSideEnum)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_guide_ball_side'",NULL);
#endif
  {
   self->set_guide_ball_side(new_guide_ball_side);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_guide_ball_side'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: num_kicks_left of class  KickerInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_num_kicks_left00
static int tolua_interfaces_KickerInterface_num_kicks_left00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"KickerInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface* self = (KickerInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'num_kicks_left'",NULL);
#endif
  {
   int tolua_ret = (int)  self->num_kicks_left();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'num_kicks_left'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_num_kicks_left of class  KickerInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_set_num_kicks_left00
static int tolua_interfaces_KickerInterface_set_num_kicks_left00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"KickerInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface* self = (KickerInterface*)  tolua_tousertype(tolua_S,1,0);
  const int new_num_kicks_left = ((const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_num_kicks_left'",NULL);
#endif
  {
   self->set_num_kicks_left(new_num_kicks_left);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_num_kicks_left'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: num_kicks_center of class  KickerInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_num_kicks_center00
static int tolua_interfaces_KickerInterface_num_kicks_center00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"KickerInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface* self = (KickerInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'num_kicks_center'",NULL);
#endif
  {
   int tolua_ret = (int)  self->num_kicks_center();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'num_kicks_center'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_num_kicks_center of class  KickerInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_set_num_kicks_center00
static int tolua_interfaces_KickerInterface_set_num_kicks_center00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"KickerInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface* self = (KickerInterface*)  tolua_tousertype(tolua_S,1,0);
  const int new_num_kicks_center = ((const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_num_kicks_center'",NULL);
#endif
  {
   self->set_num_kicks_center(new_num_kicks_center);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_num_kicks_center'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: num_kicks_right of class  KickerInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_num_kicks_right00
static int tolua_interfaces_KickerInterface_num_kicks_right00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"KickerInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface* self = (KickerInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'num_kicks_right'",NULL);
#endif
  {
   int tolua_ret = (int)  self->num_kicks_right();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'num_kicks_right'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_num_kicks_right of class  KickerInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_set_num_kicks_right00
static int tolua_interfaces_KickerInterface_set_num_kicks_right00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"KickerInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface* self = (KickerInterface*)  tolua_tousertype(tolua_S,1,0);
  const int new_num_kicks_right = ((const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_num_kicks_right'",NULL);
#endif
  {
   self->set_num_kicks_right(new_num_kicks_right);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_num_kicks_right'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: guide_ball_side of class  KickerInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_guide_ball_side00
static int tolua_interfaces_KickerInterface_guide_ball_side00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"KickerInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface* self = (KickerInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'guide_ball_side'",NULL);
#endif
  {
   KickerInterface::GuideBallSideEnum tolua_ret = (KickerInterface::GuideBallSideEnum)  self->guide_ball_side();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'guide_ball_side'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_guide_ball_side of class  KickerInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_set_guide_ball_side00
static int tolua_interfaces_KickerInterface_set_guide_ball_side00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"KickerInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface* self = (KickerInterface*)  tolua_tousertype(tolua_S,1,0);
  const KickerInterface::GuideBallSideEnum new_guide_ball_side = ((const KickerInterface::GuideBallSideEnum)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_guide_ball_side'",NULL);
#endif
  {
   self->set_guide_ball_side(new_guide_ball_side);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_guide_ball_side'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: current_intensity of class  KickerInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_current_intensity00
static int tolua_interfaces_KickerInterface_current_intensity00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"KickerInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface* self = (KickerInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'current_intensity'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->current_intensity();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'current_intensity'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_current_intensity of class  KickerInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_KickerInterface_set_current_intensity00
static int tolua_interfaces_KickerInterface_set_current_intensity00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"KickerInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  KickerInterface* self = (KickerInterface*)  tolua_tousertype(tolua_S,1,0);
  unsigned const int new_current_intensity = ((unsigned const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_current_intensity'",NULL);
#endif
  {
   self->set_current_intensity(new_current_intensity);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_current_intensity'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* get function: MOTOR_ENABLED of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_get_MotorInterface_unsigned_MOTOR_ENABLED
static int tolua_get_MotorInterface_unsigned_MOTOR_ENABLED(lua_State* tolua_S)
{
  tolua_pushnumber(tolua_S,(lua_Number)MotorInterface::MOTOR_ENABLED);
 return 1;
}
#endif //#ifndef TOLUA_DISABLE

/* get function: MOTOR_DISABLED of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_get_MotorInterface_unsigned_MOTOR_DISABLED
static int tolua_get_MotorInterface_unsigned_MOTOR_DISABLED(lua_State* tolua_S)
{
  tolua_pushnumber(tolua_S,(lua_Number)MotorInterface::MOTOR_DISABLED);
 return 1;
}
#endif //#ifndef TOLUA_DISABLE

/* get function: DRIVE_MODE_RPM of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_get_MotorInterface_unsigned_DRIVE_MODE_RPM
static int tolua_get_MotorInterface_unsigned_DRIVE_MODE_RPM(lua_State* tolua_S)
{
  tolua_pushnumber(tolua_S,(lua_Number)MotorInterface::DRIVE_MODE_RPM);
 return 1;
}
#endif //#ifndef TOLUA_DISABLE

/* get function: DRIVE_MODE_TRANS of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_get_MotorInterface_unsigned_DRIVE_MODE_TRANS
static int tolua_get_MotorInterface_unsigned_DRIVE_MODE_TRANS(lua_State* tolua_S)
{
  tolua_pushnumber(tolua_S,(lua_Number)MotorInterface::DRIVE_MODE_TRANS);
 return 1;
}
#endif //#ifndef TOLUA_DISABLE

/* get function: DRIVE_MODE_ROT of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_get_MotorInterface_unsigned_DRIVE_MODE_ROT
static int tolua_get_MotorInterface_unsigned_DRIVE_MODE_ROT(lua_State* tolua_S)
{
  tolua_pushnumber(tolua_S,(lua_Number)MotorInterface::DRIVE_MODE_ROT);
 return 1;
}
#endif //#ifndef TOLUA_DISABLE

/* get function: DRIVE_MODE_TRANS_ROT of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_get_MotorInterface_unsigned_DRIVE_MODE_TRANS_ROT
static int tolua_get_MotorInterface_unsigned_DRIVE_MODE_TRANS_ROT(lua_State* tolua_S)
{
  tolua_pushnumber(tolua_S,(lua_Number)MotorInterface::DRIVE_MODE_TRANS_ROT);
 return 1;
}
#endif //#ifndef TOLUA_DISABLE

/* get function: DRIVE_MODE_ORBIT of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_get_MotorInterface_unsigned_DRIVE_MODE_ORBIT
static int tolua_get_MotorInterface_unsigned_DRIVE_MODE_ORBIT(lua_State* tolua_S)
{
  tolua_pushnumber(tolua_S,(lua_Number)MotorInterface::DRIVE_MODE_ORBIT);
 return 1;
}
#endif //#ifndef TOLUA_DISABLE

/* get function: DRIVE_MODE_LINE_TRANS_ROT of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_get_MotorInterface_unsigned_DRIVE_MODE_LINE_TRANS_ROT
static int tolua_get_MotorInterface_unsigned_DRIVE_MODE_LINE_TRANS_ROT(lua_State* tolua_S)
{
  tolua_pushnumber(tolua_S,(lua_Number)MotorInterface::DRIVE_MODE_LINE_TRANS_ROT);
 return 1;
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  SetMotorStateMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_SetMotorStateMessage_new00
static int tolua_interfaces_MotorInterface_SetMotorStateMessage_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::SetMotorStateMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  unsigned int ini_motor_state = ((unsigned int)  tolua_tonumber(tolua_S,2,0));
  {
   MotorInterface::SetMotorStateMessage* tolua_ret = (MotorInterface::SetMotorStateMessage*)  new MotorInterface::SetMotorStateMessage(ini_motor_state);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"MotorInterface::SetMotorStateMessage");
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

/* method: new_local of class  SetMotorStateMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_SetMotorStateMessage_new00_local
static int tolua_interfaces_MotorInterface_SetMotorStateMessage_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::SetMotorStateMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  unsigned int ini_motor_state = ((unsigned int)  tolua_tonumber(tolua_S,2,0));
  {
   MotorInterface::SetMotorStateMessage* tolua_ret = (MotorInterface::SetMotorStateMessage*)  new MotorInterface::SetMotorStateMessage(ini_motor_state);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"MotorInterface::SetMotorStateMessage");
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

/* method: new of class  SetMotorStateMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_SetMotorStateMessage_new01
static int tolua_interfaces_MotorInterface_SetMotorStateMessage_new01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::SetMotorStateMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   MotorInterface::SetMotorStateMessage* tolua_ret = (MotorInterface::SetMotorStateMessage*)  new MotorInterface::SetMotorStateMessage();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"MotorInterface::SetMotorStateMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_MotorInterface_SetMotorStateMessage_new00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  SetMotorStateMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_SetMotorStateMessage_new01_local
static int tolua_interfaces_MotorInterface_SetMotorStateMessage_new01_local(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::SetMotorStateMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   MotorInterface::SetMotorStateMessage* tolua_ret = (MotorInterface::SetMotorStateMessage*)  new MotorInterface::SetMotorStateMessage();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"MotorInterface::SetMotorStateMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_MotorInterface_SetMotorStateMessage_new00_local(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: delete of class  SetMotorStateMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_SetMotorStateMessage_delete00
static int tolua_interfaces_MotorInterface_SetMotorStateMessage_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::SetMotorStateMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::SetMotorStateMessage* self = (MotorInterface::SetMotorStateMessage*)  tolua_tousertype(tolua_S,1,0);
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

/* method: motor_state of class  SetMotorStateMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_SetMotorStateMessage_motor_state00
static int tolua_interfaces_MotorInterface_SetMotorStateMessage_motor_state00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::SetMotorStateMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::SetMotorStateMessage* self = (MotorInterface::SetMotorStateMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'motor_state'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->motor_state();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'motor_state'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_motor_state of class  SetMotorStateMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_SetMotorStateMessage_set_motor_state00
static int tolua_interfaces_MotorInterface_SetMotorStateMessage_set_motor_state00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::SetMotorStateMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::SetMotorStateMessage* self = (MotorInterface::SetMotorStateMessage*)  tolua_tousertype(tolua_S,1,0);
  unsigned const int new_motor_state = ((unsigned const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_motor_state'",NULL);
#endif
  {
   self->set_motor_state(new_motor_state);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_motor_state'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  AcquireControlMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_AcquireControlMessage_new00
static int tolua_interfaces_MotorInterface_AcquireControlMessage_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::AcquireControlMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isstring(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  unsigned long int ini_thread_id = ((unsigned long int)  tolua_tonumber(tolua_S,2,0));
  char* ini_thread_name = ((char*)  tolua_tostring(tolua_S,3,0));
  {
   MotorInterface::AcquireControlMessage* tolua_ret = (MotorInterface::AcquireControlMessage*)  new MotorInterface::AcquireControlMessage(ini_thread_id,ini_thread_name);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"MotorInterface::AcquireControlMessage");
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

/* method: new_local of class  AcquireControlMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_AcquireControlMessage_new00_local
static int tolua_interfaces_MotorInterface_AcquireControlMessage_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::AcquireControlMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isstring(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  unsigned long int ini_thread_id = ((unsigned long int)  tolua_tonumber(tolua_S,2,0));
  char* ini_thread_name = ((char*)  tolua_tostring(tolua_S,3,0));
  {
   MotorInterface::AcquireControlMessage* tolua_ret = (MotorInterface::AcquireControlMessage*)  new MotorInterface::AcquireControlMessage(ini_thread_id,ini_thread_name);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"MotorInterface::AcquireControlMessage");
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

/* method: new of class  AcquireControlMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_AcquireControlMessage_new01
static int tolua_interfaces_MotorInterface_AcquireControlMessage_new01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::AcquireControlMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   MotorInterface::AcquireControlMessage* tolua_ret = (MotorInterface::AcquireControlMessage*)  new MotorInterface::AcquireControlMessage();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"MotorInterface::AcquireControlMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_MotorInterface_AcquireControlMessage_new00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  AcquireControlMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_AcquireControlMessage_new01_local
static int tolua_interfaces_MotorInterface_AcquireControlMessage_new01_local(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::AcquireControlMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   MotorInterface::AcquireControlMessage* tolua_ret = (MotorInterface::AcquireControlMessage*)  new MotorInterface::AcquireControlMessage();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"MotorInterface::AcquireControlMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_MotorInterface_AcquireControlMessage_new00_local(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: delete of class  AcquireControlMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_AcquireControlMessage_delete00
static int tolua_interfaces_MotorInterface_AcquireControlMessage_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::AcquireControlMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::AcquireControlMessage* self = (MotorInterface::AcquireControlMessage*)  tolua_tousertype(tolua_S,1,0);
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

/* method: thread_id of class  AcquireControlMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_AcquireControlMessage_thread_id00
static int tolua_interfaces_MotorInterface_AcquireControlMessage_thread_id00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::AcquireControlMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::AcquireControlMessage* self = (MotorInterface::AcquireControlMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'thread_id'",NULL);
#endif
  {
   unsigned long int tolua_ret = (unsigned long int)  self->thread_id();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'thread_id'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_thread_id of class  AcquireControlMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_AcquireControlMessage_set_thread_id00
static int tolua_interfaces_MotorInterface_AcquireControlMessage_set_thread_id00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::AcquireControlMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::AcquireControlMessage* self = (MotorInterface::AcquireControlMessage*)  tolua_tousertype(tolua_S,1,0);
  unsigned long const int new_thread_id = ((unsigned long const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_thread_id'",NULL);
#endif
  {
   self->set_thread_id(new_thread_id);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_thread_id'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: thread_name of class  AcquireControlMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_AcquireControlMessage_thread_name00
static int tolua_interfaces_MotorInterface_AcquireControlMessage_thread_name00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::AcquireControlMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::AcquireControlMessage* self = (MotorInterface::AcquireControlMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'thread_name'",NULL);
#endif
  {
   char* tolua_ret = (char*)  self->thread_name();
   tolua_pushstring(tolua_S,(const char*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'thread_name'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_thread_name of class  AcquireControlMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_AcquireControlMessage_set_thread_name00
static int tolua_interfaces_MotorInterface_AcquireControlMessage_set_thread_name00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::AcquireControlMessage",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::AcquireControlMessage* self = (MotorInterface::AcquireControlMessage*)  tolua_tousertype(tolua_S,1,0);
  const char* new_thread_name = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_thread_name'",NULL);
#endif
  {
   self->set_thread_name(new_thread_name);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_thread_name'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  ResetOdometryMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_ResetOdometryMessage_new00
static int tolua_interfaces_MotorInterface_ResetOdometryMessage_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::ResetOdometryMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   MotorInterface::ResetOdometryMessage* tolua_ret = (MotorInterface::ResetOdometryMessage*)  new MotorInterface::ResetOdometryMessage();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"MotorInterface::ResetOdometryMessage");
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

/* method: new_local of class  ResetOdometryMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_ResetOdometryMessage_new00_local
static int tolua_interfaces_MotorInterface_ResetOdometryMessage_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::ResetOdometryMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   MotorInterface::ResetOdometryMessage* tolua_ret = (MotorInterface::ResetOdometryMessage*)  new MotorInterface::ResetOdometryMessage();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"MotorInterface::ResetOdometryMessage");
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

/* method: delete of class  ResetOdometryMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_ResetOdometryMessage_delete00
static int tolua_interfaces_MotorInterface_ResetOdometryMessage_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::ResetOdometryMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::ResetOdometryMessage* self = (MotorInterface::ResetOdometryMessage*)  tolua_tousertype(tolua_S,1,0);
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

/* method: new of class  DriveRPMMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_DriveRPMMessage_new00
static int tolua_interfaces_MotorInterface_DriveRPMMessage_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::DriveRPMMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  float ini_front_right = ((float)  tolua_tonumber(tolua_S,2,0));
  float ini_front_left = ((float)  tolua_tonumber(tolua_S,3,0));
  float ini_rear = ((float)  tolua_tonumber(tolua_S,4,0));
  {
   MotorInterface::DriveRPMMessage* tolua_ret = (MotorInterface::DriveRPMMessage*)  new MotorInterface::DriveRPMMessage(ini_front_right,ini_front_left,ini_rear);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"MotorInterface::DriveRPMMessage");
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

/* method: new_local of class  DriveRPMMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_DriveRPMMessage_new00_local
static int tolua_interfaces_MotorInterface_DriveRPMMessage_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::DriveRPMMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  float ini_front_right = ((float)  tolua_tonumber(tolua_S,2,0));
  float ini_front_left = ((float)  tolua_tonumber(tolua_S,3,0));
  float ini_rear = ((float)  tolua_tonumber(tolua_S,4,0));
  {
   MotorInterface::DriveRPMMessage* tolua_ret = (MotorInterface::DriveRPMMessage*)  new MotorInterface::DriveRPMMessage(ini_front_right,ini_front_left,ini_rear);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"MotorInterface::DriveRPMMessage");
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

/* method: new of class  DriveRPMMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_DriveRPMMessage_new01
static int tolua_interfaces_MotorInterface_DriveRPMMessage_new01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::DriveRPMMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   MotorInterface::DriveRPMMessage* tolua_ret = (MotorInterface::DriveRPMMessage*)  new MotorInterface::DriveRPMMessage();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"MotorInterface::DriveRPMMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_MotorInterface_DriveRPMMessage_new00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  DriveRPMMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_DriveRPMMessage_new01_local
static int tolua_interfaces_MotorInterface_DriveRPMMessage_new01_local(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::DriveRPMMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   MotorInterface::DriveRPMMessage* tolua_ret = (MotorInterface::DriveRPMMessage*)  new MotorInterface::DriveRPMMessage();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"MotorInterface::DriveRPMMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_MotorInterface_DriveRPMMessage_new00_local(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: delete of class  DriveRPMMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_DriveRPMMessage_delete00
static int tolua_interfaces_MotorInterface_DriveRPMMessage_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::DriveRPMMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::DriveRPMMessage* self = (MotorInterface::DriveRPMMessage*)  tolua_tousertype(tolua_S,1,0);
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

/* method: front_right of class  DriveRPMMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_DriveRPMMessage_front_right00
static int tolua_interfaces_MotorInterface_DriveRPMMessage_front_right00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::DriveRPMMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::DriveRPMMessage* self = (MotorInterface::DriveRPMMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'front_right'",NULL);
#endif
  {
   float tolua_ret = (float)  self->front_right();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'front_right'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_front_right of class  DriveRPMMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_DriveRPMMessage_set_front_right00
static int tolua_interfaces_MotorInterface_DriveRPMMessage_set_front_right00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::DriveRPMMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::DriveRPMMessage* self = (MotorInterface::DriveRPMMessage*)  tolua_tousertype(tolua_S,1,0);
  const float new_front_right = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_front_right'",NULL);
#endif
  {
   self->set_front_right(new_front_right);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_front_right'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: front_left of class  DriveRPMMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_DriveRPMMessage_front_left00
static int tolua_interfaces_MotorInterface_DriveRPMMessage_front_left00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::DriveRPMMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::DriveRPMMessage* self = (MotorInterface::DriveRPMMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'front_left'",NULL);
#endif
  {
   float tolua_ret = (float)  self->front_left();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'front_left'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_front_left of class  DriveRPMMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_DriveRPMMessage_set_front_left00
static int tolua_interfaces_MotorInterface_DriveRPMMessage_set_front_left00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::DriveRPMMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::DriveRPMMessage* self = (MotorInterface::DriveRPMMessage*)  tolua_tousertype(tolua_S,1,0);
  const float new_front_left = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_front_left'",NULL);
#endif
  {
   self->set_front_left(new_front_left);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_front_left'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: rear of class  DriveRPMMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_DriveRPMMessage_rear00
static int tolua_interfaces_MotorInterface_DriveRPMMessage_rear00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::DriveRPMMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::DriveRPMMessage* self = (MotorInterface::DriveRPMMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'rear'",NULL);
#endif
  {
   float tolua_ret = (float)  self->rear();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'rear'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_rear of class  DriveRPMMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_DriveRPMMessage_set_rear00
static int tolua_interfaces_MotorInterface_DriveRPMMessage_set_rear00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::DriveRPMMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::DriveRPMMessage* self = (MotorInterface::DriveRPMMessage*)  tolua_tousertype(tolua_S,1,0);
  const float new_rear = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_rear'",NULL);
#endif
  {
   self->set_rear(new_rear);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_rear'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  TransMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_TransMessage_new00
static int tolua_interfaces_MotorInterface_TransMessage_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::TransMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  float ini_vx = ((float)  tolua_tonumber(tolua_S,2,0));
  float ini_vy = ((float)  tolua_tonumber(tolua_S,3,0));
  {
   MotorInterface::TransMessage* tolua_ret = (MotorInterface::TransMessage*)  new MotorInterface::TransMessage(ini_vx,ini_vy);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"MotorInterface::TransMessage");
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

/* method: new_local of class  TransMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_TransMessage_new00_local
static int tolua_interfaces_MotorInterface_TransMessage_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::TransMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  float ini_vx = ((float)  tolua_tonumber(tolua_S,2,0));
  float ini_vy = ((float)  tolua_tonumber(tolua_S,3,0));
  {
   MotorInterface::TransMessage* tolua_ret = (MotorInterface::TransMessage*)  new MotorInterface::TransMessage(ini_vx,ini_vy);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"MotorInterface::TransMessage");
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

/* method: new of class  TransMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_TransMessage_new01
static int tolua_interfaces_MotorInterface_TransMessage_new01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::TransMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   MotorInterface::TransMessage* tolua_ret = (MotorInterface::TransMessage*)  new MotorInterface::TransMessage();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"MotorInterface::TransMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_MotorInterface_TransMessage_new00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  TransMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_TransMessage_new01_local
static int tolua_interfaces_MotorInterface_TransMessage_new01_local(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::TransMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   MotorInterface::TransMessage* tolua_ret = (MotorInterface::TransMessage*)  new MotorInterface::TransMessage();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"MotorInterface::TransMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_MotorInterface_TransMessage_new00_local(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: delete of class  TransMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_TransMessage_delete00
static int tolua_interfaces_MotorInterface_TransMessage_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::TransMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::TransMessage* self = (MotorInterface::TransMessage*)  tolua_tousertype(tolua_S,1,0);
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

/* method: vx of class  TransMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_TransMessage_vx00
static int tolua_interfaces_MotorInterface_TransMessage_vx00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::TransMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::TransMessage* self = (MotorInterface::TransMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'vx'",NULL);
#endif
  {
   float tolua_ret = (float)  self->vx();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'vx'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_vx of class  TransMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_TransMessage_set_vx00
static int tolua_interfaces_MotorInterface_TransMessage_set_vx00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::TransMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::TransMessage* self = (MotorInterface::TransMessage*)  tolua_tousertype(tolua_S,1,0);
  const float new_vx = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_vx'",NULL);
#endif
  {
   self->set_vx(new_vx);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_vx'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: vy of class  TransMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_TransMessage_vy00
static int tolua_interfaces_MotorInterface_TransMessage_vy00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::TransMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::TransMessage* self = (MotorInterface::TransMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'vy'",NULL);
#endif
  {
   float tolua_ret = (float)  self->vy();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'vy'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_vy of class  TransMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_TransMessage_set_vy00
static int tolua_interfaces_MotorInterface_TransMessage_set_vy00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::TransMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::TransMessage* self = (MotorInterface::TransMessage*)  tolua_tousertype(tolua_S,1,0);
  const float new_vy = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_vy'",NULL);
#endif
  {
   self->set_vy(new_vy);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_vy'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  RotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_RotMessage_new00
static int tolua_interfaces_MotorInterface_RotMessage_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::RotMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  float ini_omega = ((float)  tolua_tonumber(tolua_S,2,0));
  {
   MotorInterface::RotMessage* tolua_ret = (MotorInterface::RotMessage*)  new MotorInterface::RotMessage(ini_omega);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"MotorInterface::RotMessage");
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

/* method: new_local of class  RotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_RotMessage_new00_local
static int tolua_interfaces_MotorInterface_RotMessage_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::RotMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  float ini_omega = ((float)  tolua_tonumber(tolua_S,2,0));
  {
   MotorInterface::RotMessage* tolua_ret = (MotorInterface::RotMessage*)  new MotorInterface::RotMessage(ini_omega);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"MotorInterface::RotMessage");
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

/* method: new of class  RotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_RotMessage_new01
static int tolua_interfaces_MotorInterface_RotMessage_new01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::RotMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   MotorInterface::RotMessage* tolua_ret = (MotorInterface::RotMessage*)  new MotorInterface::RotMessage();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"MotorInterface::RotMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_MotorInterface_RotMessage_new00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  RotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_RotMessage_new01_local
static int tolua_interfaces_MotorInterface_RotMessage_new01_local(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::RotMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   MotorInterface::RotMessage* tolua_ret = (MotorInterface::RotMessage*)  new MotorInterface::RotMessage();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"MotorInterface::RotMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_MotorInterface_RotMessage_new00_local(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: delete of class  RotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_RotMessage_delete00
static int tolua_interfaces_MotorInterface_RotMessage_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::RotMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::RotMessage* self = (MotorInterface::RotMessage*)  tolua_tousertype(tolua_S,1,0);
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

/* method: omega of class  RotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_RotMessage_omega00
static int tolua_interfaces_MotorInterface_RotMessage_omega00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::RotMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::RotMessage* self = (MotorInterface::RotMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'omega'",NULL);
#endif
  {
   float tolua_ret = (float)  self->omega();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'omega'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_omega of class  RotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_RotMessage_set_omega00
static int tolua_interfaces_MotorInterface_RotMessage_set_omega00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::RotMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::RotMessage* self = (MotorInterface::RotMessage*)  tolua_tousertype(tolua_S,1,0);
  const float new_omega = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_omega'",NULL);
#endif
  {
   self->set_omega(new_omega);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_omega'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  TransRotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_TransRotMessage_new00
static int tolua_interfaces_MotorInterface_TransRotMessage_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::TransRotMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  float ini_vx = ((float)  tolua_tonumber(tolua_S,2,0));
  float ini_vy = ((float)  tolua_tonumber(tolua_S,3,0));
  float ini_omega = ((float)  tolua_tonumber(tolua_S,4,0));
  {
   MotorInterface::TransRotMessage* tolua_ret = (MotorInterface::TransRotMessage*)  new MotorInterface::TransRotMessage(ini_vx,ini_vy,ini_omega);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"MotorInterface::TransRotMessage");
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

/* method: new_local of class  TransRotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_TransRotMessage_new00_local
static int tolua_interfaces_MotorInterface_TransRotMessage_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::TransRotMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  float ini_vx = ((float)  tolua_tonumber(tolua_S,2,0));
  float ini_vy = ((float)  tolua_tonumber(tolua_S,3,0));
  float ini_omega = ((float)  tolua_tonumber(tolua_S,4,0));
  {
   MotorInterface::TransRotMessage* tolua_ret = (MotorInterface::TransRotMessage*)  new MotorInterface::TransRotMessage(ini_vx,ini_vy,ini_omega);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"MotorInterface::TransRotMessage");
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

/* method: new of class  TransRotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_TransRotMessage_new01
static int tolua_interfaces_MotorInterface_TransRotMessage_new01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::TransRotMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   MotorInterface::TransRotMessage* tolua_ret = (MotorInterface::TransRotMessage*)  new MotorInterface::TransRotMessage();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"MotorInterface::TransRotMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_MotorInterface_TransRotMessage_new00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  TransRotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_TransRotMessage_new01_local
static int tolua_interfaces_MotorInterface_TransRotMessage_new01_local(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::TransRotMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   MotorInterface::TransRotMessage* tolua_ret = (MotorInterface::TransRotMessage*)  new MotorInterface::TransRotMessage();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"MotorInterface::TransRotMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_MotorInterface_TransRotMessage_new00_local(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: delete of class  TransRotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_TransRotMessage_delete00
static int tolua_interfaces_MotorInterface_TransRotMessage_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::TransRotMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::TransRotMessage* self = (MotorInterface::TransRotMessage*)  tolua_tousertype(tolua_S,1,0);
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

/* method: vx of class  TransRotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_TransRotMessage_vx00
static int tolua_interfaces_MotorInterface_TransRotMessage_vx00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::TransRotMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::TransRotMessage* self = (MotorInterface::TransRotMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'vx'",NULL);
#endif
  {
   float tolua_ret = (float)  self->vx();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'vx'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_vx of class  TransRotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_TransRotMessage_set_vx00
static int tolua_interfaces_MotorInterface_TransRotMessage_set_vx00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::TransRotMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::TransRotMessage* self = (MotorInterface::TransRotMessage*)  tolua_tousertype(tolua_S,1,0);
  const float new_vx = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_vx'",NULL);
#endif
  {
   self->set_vx(new_vx);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_vx'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: vy of class  TransRotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_TransRotMessage_vy00
static int tolua_interfaces_MotorInterface_TransRotMessage_vy00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::TransRotMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::TransRotMessage* self = (MotorInterface::TransRotMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'vy'",NULL);
#endif
  {
   float tolua_ret = (float)  self->vy();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'vy'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_vy of class  TransRotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_TransRotMessage_set_vy00
static int tolua_interfaces_MotorInterface_TransRotMessage_set_vy00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::TransRotMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::TransRotMessage* self = (MotorInterface::TransRotMessage*)  tolua_tousertype(tolua_S,1,0);
  const float new_vy = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_vy'",NULL);
#endif
  {
   self->set_vy(new_vy);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_vy'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: omega of class  TransRotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_TransRotMessage_omega00
static int tolua_interfaces_MotorInterface_TransRotMessage_omega00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::TransRotMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::TransRotMessage* self = (MotorInterface::TransRotMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'omega'",NULL);
#endif
  {
   float tolua_ret = (float)  self->omega();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'omega'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_omega of class  TransRotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_TransRotMessage_set_omega00
static int tolua_interfaces_MotorInterface_TransRotMessage_set_omega00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::TransRotMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::TransRotMessage* self = (MotorInterface::TransRotMessage*)  tolua_tousertype(tolua_S,1,0);
  const float new_omega = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_omega'",NULL);
#endif
  {
   self->set_omega(new_omega);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_omega'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  OrbitMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_OrbitMessage_new00
static int tolua_interfaces_MotorInterface_OrbitMessage_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::OrbitMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  float ini_px = ((float)  tolua_tonumber(tolua_S,2,0));
  float ini_py = ((float)  tolua_tonumber(tolua_S,3,0));
  float ini_omega = ((float)  tolua_tonumber(tolua_S,4,0));
  {
   MotorInterface::OrbitMessage* tolua_ret = (MotorInterface::OrbitMessage*)  new MotorInterface::OrbitMessage(ini_px,ini_py,ini_omega);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"MotorInterface::OrbitMessage");
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

/* method: new_local of class  OrbitMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_OrbitMessage_new00_local
static int tolua_interfaces_MotorInterface_OrbitMessage_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::OrbitMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  float ini_px = ((float)  tolua_tonumber(tolua_S,2,0));
  float ini_py = ((float)  tolua_tonumber(tolua_S,3,0));
  float ini_omega = ((float)  tolua_tonumber(tolua_S,4,0));
  {
   MotorInterface::OrbitMessage* tolua_ret = (MotorInterface::OrbitMessage*)  new MotorInterface::OrbitMessage(ini_px,ini_py,ini_omega);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"MotorInterface::OrbitMessage");
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

/* method: new of class  OrbitMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_OrbitMessage_new01
static int tolua_interfaces_MotorInterface_OrbitMessage_new01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::OrbitMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   MotorInterface::OrbitMessage* tolua_ret = (MotorInterface::OrbitMessage*)  new MotorInterface::OrbitMessage();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"MotorInterface::OrbitMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_MotorInterface_OrbitMessage_new00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  OrbitMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_OrbitMessage_new01_local
static int tolua_interfaces_MotorInterface_OrbitMessage_new01_local(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::OrbitMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   MotorInterface::OrbitMessage* tolua_ret = (MotorInterface::OrbitMessage*)  new MotorInterface::OrbitMessage();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"MotorInterface::OrbitMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_MotorInterface_OrbitMessage_new00_local(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: delete of class  OrbitMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_OrbitMessage_delete00
static int tolua_interfaces_MotorInterface_OrbitMessage_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::OrbitMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::OrbitMessage* self = (MotorInterface::OrbitMessage*)  tolua_tousertype(tolua_S,1,0);
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

/* method: px of class  OrbitMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_OrbitMessage_px00
static int tolua_interfaces_MotorInterface_OrbitMessage_px00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::OrbitMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::OrbitMessage* self = (MotorInterface::OrbitMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'px'",NULL);
#endif
  {
   float tolua_ret = (float)  self->px();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'px'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_px of class  OrbitMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_OrbitMessage_set_px00
static int tolua_interfaces_MotorInterface_OrbitMessage_set_px00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::OrbitMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::OrbitMessage* self = (MotorInterface::OrbitMessage*)  tolua_tousertype(tolua_S,1,0);
  const float new_px = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_px'",NULL);
#endif
  {
   self->set_px(new_px);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_px'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: py of class  OrbitMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_OrbitMessage_py00
static int tolua_interfaces_MotorInterface_OrbitMessage_py00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::OrbitMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::OrbitMessage* self = (MotorInterface::OrbitMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'py'",NULL);
#endif
  {
   float tolua_ret = (float)  self->py();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'py'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_py of class  OrbitMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_OrbitMessage_set_py00
static int tolua_interfaces_MotorInterface_OrbitMessage_set_py00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::OrbitMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::OrbitMessage* self = (MotorInterface::OrbitMessage*)  tolua_tousertype(tolua_S,1,0);
  const float new_py = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_py'",NULL);
#endif
  {
   self->set_py(new_py);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_py'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: omega of class  OrbitMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_OrbitMessage_omega00
static int tolua_interfaces_MotorInterface_OrbitMessage_omega00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::OrbitMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::OrbitMessage* self = (MotorInterface::OrbitMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'omega'",NULL);
#endif
  {
   float tolua_ret = (float)  self->omega();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'omega'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_omega of class  OrbitMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_OrbitMessage_set_omega00
static int tolua_interfaces_MotorInterface_OrbitMessage_set_omega00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::OrbitMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::OrbitMessage* self = (MotorInterface::OrbitMessage*)  tolua_tousertype(tolua_S,1,0);
  const float new_omega = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_omega'",NULL);
#endif
  {
   self->set_omega(new_omega);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_omega'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  LinTransRotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_LinTransRotMessage_new00
static int tolua_interfaces_MotorInterface_LinTransRotMessage_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::LinTransRotMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  float ini_vx = ((float)  tolua_tonumber(tolua_S,2,0));
  float ini_vy = ((float)  tolua_tonumber(tolua_S,3,0));
  float ini_omega = ((float)  tolua_tonumber(tolua_S,4,0));
  {
   MotorInterface::LinTransRotMessage* tolua_ret = (MotorInterface::LinTransRotMessage*)  new MotorInterface::LinTransRotMessage(ini_vx,ini_vy,ini_omega);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"MotorInterface::LinTransRotMessage");
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

/* method: new_local of class  LinTransRotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_LinTransRotMessage_new00_local
static int tolua_interfaces_MotorInterface_LinTransRotMessage_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::LinTransRotMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  float ini_vx = ((float)  tolua_tonumber(tolua_S,2,0));
  float ini_vy = ((float)  tolua_tonumber(tolua_S,3,0));
  float ini_omega = ((float)  tolua_tonumber(tolua_S,4,0));
  {
   MotorInterface::LinTransRotMessage* tolua_ret = (MotorInterface::LinTransRotMessage*)  new MotorInterface::LinTransRotMessage(ini_vx,ini_vy,ini_omega);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"MotorInterface::LinTransRotMessage");
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

/* method: new of class  LinTransRotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_LinTransRotMessage_new01
static int tolua_interfaces_MotorInterface_LinTransRotMessage_new01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::LinTransRotMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   MotorInterface::LinTransRotMessage* tolua_ret = (MotorInterface::LinTransRotMessage*)  new MotorInterface::LinTransRotMessage();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"MotorInterface::LinTransRotMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_MotorInterface_LinTransRotMessage_new00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  LinTransRotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_LinTransRotMessage_new01_local
static int tolua_interfaces_MotorInterface_LinTransRotMessage_new01_local(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MotorInterface::LinTransRotMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   MotorInterface::LinTransRotMessage* tolua_ret = (MotorInterface::LinTransRotMessage*)  new MotorInterface::LinTransRotMessage();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"MotorInterface::LinTransRotMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_MotorInterface_LinTransRotMessage_new00_local(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: delete of class  LinTransRotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_LinTransRotMessage_delete00
static int tolua_interfaces_MotorInterface_LinTransRotMessage_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::LinTransRotMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::LinTransRotMessage* self = (MotorInterface::LinTransRotMessage*)  tolua_tousertype(tolua_S,1,0);
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

/* method: vx of class  LinTransRotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_LinTransRotMessage_vx00
static int tolua_interfaces_MotorInterface_LinTransRotMessage_vx00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::LinTransRotMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::LinTransRotMessage* self = (MotorInterface::LinTransRotMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'vx'",NULL);
#endif
  {
   float tolua_ret = (float)  self->vx();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'vx'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_vx of class  LinTransRotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_LinTransRotMessage_set_vx00
static int tolua_interfaces_MotorInterface_LinTransRotMessage_set_vx00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::LinTransRotMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::LinTransRotMessage* self = (MotorInterface::LinTransRotMessage*)  tolua_tousertype(tolua_S,1,0);
  const float new_vx = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_vx'",NULL);
#endif
  {
   self->set_vx(new_vx);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_vx'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: vy of class  LinTransRotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_LinTransRotMessage_vy00
static int tolua_interfaces_MotorInterface_LinTransRotMessage_vy00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::LinTransRotMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::LinTransRotMessage* self = (MotorInterface::LinTransRotMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'vy'",NULL);
#endif
  {
   float tolua_ret = (float)  self->vy();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'vy'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_vy of class  LinTransRotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_LinTransRotMessage_set_vy00
static int tolua_interfaces_MotorInterface_LinTransRotMessage_set_vy00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::LinTransRotMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::LinTransRotMessage* self = (MotorInterface::LinTransRotMessage*)  tolua_tousertype(tolua_S,1,0);
  const float new_vy = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_vy'",NULL);
#endif
  {
   self->set_vy(new_vy);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_vy'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: omega of class  LinTransRotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_LinTransRotMessage_omega00
static int tolua_interfaces_MotorInterface_LinTransRotMessage_omega00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::LinTransRotMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::LinTransRotMessage* self = (MotorInterface::LinTransRotMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'omega'",NULL);
#endif
  {
   float tolua_ret = (float)  self->omega();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'omega'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_omega of class  LinTransRotMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_LinTransRotMessage_set_omega00
static int tolua_interfaces_MotorInterface_LinTransRotMessage_set_omega00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface::LinTransRotMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface::LinTransRotMessage* self = (MotorInterface::LinTransRotMessage*)  tolua_tousertype(tolua_S,1,0);
  const float new_omega = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_omega'",NULL);
#endif
  {
   self->set_omega(new_omega);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_omega'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: motor_state of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_motor_state00
static int tolua_interfaces_MotorInterface_motor_state00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'motor_state'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->motor_state();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'motor_state'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_motor_state of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_set_motor_state00
static int tolua_interfaces_MotorInterface_set_motor_state00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
  unsigned const int new_motor_state = ((unsigned const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_motor_state'",NULL);
#endif
  {
   self->set_motor_state(new_motor_state);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_motor_state'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: drive_mode of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_drive_mode00
static int tolua_interfaces_MotorInterface_drive_mode00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'drive_mode'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->drive_mode();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'drive_mode'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_drive_mode of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_set_drive_mode00
static int tolua_interfaces_MotorInterface_set_drive_mode00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
  unsigned const int new_drive_mode = ((unsigned const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_drive_mode'",NULL);
#endif
  {
   self->set_drive_mode(new_drive_mode);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_drive_mode'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: right_rpm of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_right_rpm00
static int tolua_interfaces_MotorInterface_right_rpm00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'right_rpm'",NULL);
#endif
  {
   int tolua_ret = (int)  self->right_rpm();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'right_rpm'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_right_rpm of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_set_right_rpm00
static int tolua_interfaces_MotorInterface_set_right_rpm00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
  const int new_right_rpm = ((const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_right_rpm'",NULL);
#endif
  {
   self->set_right_rpm(new_right_rpm);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_right_rpm'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: rear_rpm of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_rear_rpm00
static int tolua_interfaces_MotorInterface_rear_rpm00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'rear_rpm'",NULL);
#endif
  {
   int tolua_ret = (int)  self->rear_rpm();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'rear_rpm'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_rear_rpm of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_set_rear_rpm00
static int tolua_interfaces_MotorInterface_set_rear_rpm00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
  const int new_rear_rpm = ((const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_rear_rpm'",NULL);
#endif
  {
   self->set_rear_rpm(new_rear_rpm);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_rear_rpm'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: left_rpm of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_left_rpm00
static int tolua_interfaces_MotorInterface_left_rpm00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'left_rpm'",NULL);
#endif
  {
   int tolua_ret = (int)  self->left_rpm();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'left_rpm'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_left_rpm of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_set_left_rpm00
static int tolua_interfaces_MotorInterface_set_left_rpm00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
  const int new_left_rpm = ((const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_left_rpm'",NULL);
#endif
  {
   self->set_left_rpm(new_left_rpm);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_left_rpm'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: odometry_path_length of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_odometry_path_length00
static int tolua_interfaces_MotorInterface_odometry_path_length00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'odometry_path_length'",NULL);
#endif
  {
   float tolua_ret = (float)  self->odometry_path_length();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'odometry_path_length'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_odometry_path_length of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_set_odometry_path_length00
static int tolua_interfaces_MotorInterface_set_odometry_path_length00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_odometry_path_length = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_odometry_path_length'",NULL);
#endif
  {
   self->set_odometry_path_length(new_odometry_path_length);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_odometry_path_length'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: odometry_position_x of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_odometry_position_x00
static int tolua_interfaces_MotorInterface_odometry_position_x00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'odometry_position_x'",NULL);
#endif
  {
   float tolua_ret = (float)  self->odometry_position_x();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'odometry_position_x'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_odometry_position_x of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_set_odometry_position_x00
static int tolua_interfaces_MotorInterface_set_odometry_position_x00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_odometry_position_x = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_odometry_position_x'",NULL);
#endif
  {
   self->set_odometry_position_x(new_odometry_position_x);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_odometry_position_x'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: odometry_position_y of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_odometry_position_y00
static int tolua_interfaces_MotorInterface_odometry_position_y00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'odometry_position_y'",NULL);
#endif
  {
   float tolua_ret = (float)  self->odometry_position_y();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'odometry_position_y'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_odometry_position_y of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_set_odometry_position_y00
static int tolua_interfaces_MotorInterface_set_odometry_position_y00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_odometry_position_y = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_odometry_position_y'",NULL);
#endif
  {
   self->set_odometry_position_y(new_odometry_position_y);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_odometry_position_y'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: odometry_orientation of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_odometry_orientation00
static int tolua_interfaces_MotorInterface_odometry_orientation00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'odometry_orientation'",NULL);
#endif
  {
   float tolua_ret = (float)  self->odometry_orientation();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'odometry_orientation'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_odometry_orientation of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_set_odometry_orientation00
static int tolua_interfaces_MotorInterface_set_odometry_orientation00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_odometry_orientation = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_odometry_orientation'",NULL);
#endif
  {
   self->set_odometry_orientation(new_odometry_orientation);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_odometry_orientation'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: vx of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_vx00
static int tolua_interfaces_MotorInterface_vx00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'vx'",NULL);
#endif
  {
   float tolua_ret = (float)  self->vx();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'vx'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_vx of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_set_vx00
static int tolua_interfaces_MotorInterface_set_vx00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_vx = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_vx'",NULL);
#endif
  {
   self->set_vx(new_vx);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_vx'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: vy of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_vy00
static int tolua_interfaces_MotorInterface_vy00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'vy'",NULL);
#endif
  {
   float tolua_ret = (float)  self->vy();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'vy'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_vy of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_set_vy00
static int tolua_interfaces_MotorInterface_set_vy00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_vy = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_vy'",NULL);
#endif
  {
   self->set_vy(new_vy);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_vy'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: omega of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_omega00
static int tolua_interfaces_MotorInterface_omega00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'omega'",NULL);
#endif
  {
   float tolua_ret = (float)  self->omega();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'omega'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_omega of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_set_omega00
static int tolua_interfaces_MotorInterface_set_omega00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_omega = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_omega'",NULL);
#endif
  {
   self->set_omega(new_omega);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_omega'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: controller_thread_id of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_controller_thread_id00
static int tolua_interfaces_MotorInterface_controller_thread_id00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'controller_thread_id'",NULL);
#endif
  {
   unsigned long int tolua_ret = (unsigned long int)  self->controller_thread_id();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'controller_thread_id'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_controller_thread_id of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_set_controller_thread_id00
static int tolua_interfaces_MotorInterface_set_controller_thread_id00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
  unsigned long const int new_controller_thread_id = ((unsigned long const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_controller_thread_id'",NULL);
#endif
  {
   self->set_controller_thread_id(new_controller_thread_id);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_controller_thread_id'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: controller_thread_name of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_controller_thread_name00
static int tolua_interfaces_MotorInterface_controller_thread_name00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'controller_thread_name'",NULL);
#endif
  {
   char* tolua_ret = (char*)  self->controller_thread_name();
   tolua_pushstring(tolua_S,(const char*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'controller_thread_name'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_controller_thread_name of class  MotorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_MotorInterface_set_controller_thread_name00
static int tolua_interfaces_MotorInterface_set_controller_thread_name00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MotorInterface",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MotorInterface* self = (MotorInterface*)  tolua_tousertype(tolua_S,1,0);
  const char* new_controller_thread_name = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_controller_thread_name'",NULL);
#endif
  {
   self->set_controller_thread_name(new_controller_thread_name);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_controller_thread_name'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  TargetMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_TargetMessage_new00
static int tolua_interfaces_NavigatorInterface_TargetMessage_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"NavigatorInterface::TargetMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  float ini_x = ((float)  tolua_tonumber(tolua_S,2,0));
  float ini_y = ((float)  tolua_tonumber(tolua_S,3,0));
  float ini_orientation = ((float)  tolua_tonumber(tolua_S,4,0));
  {
   NavigatorInterface::TargetMessage* tolua_ret = (NavigatorInterface::TargetMessage*)  new NavigatorInterface::TargetMessage(ini_x,ini_y,ini_orientation);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"NavigatorInterface::TargetMessage");
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

/* method: new_local of class  TargetMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_TargetMessage_new00_local
static int tolua_interfaces_NavigatorInterface_TargetMessage_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"NavigatorInterface::TargetMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  float ini_x = ((float)  tolua_tonumber(tolua_S,2,0));
  float ini_y = ((float)  tolua_tonumber(tolua_S,3,0));
  float ini_orientation = ((float)  tolua_tonumber(tolua_S,4,0));
  {
   NavigatorInterface::TargetMessage* tolua_ret = (NavigatorInterface::TargetMessage*)  new NavigatorInterface::TargetMessage(ini_x,ini_y,ini_orientation);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"NavigatorInterface::TargetMessage");
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

/* method: new of class  TargetMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_TargetMessage_new01
static int tolua_interfaces_NavigatorInterface_TargetMessage_new01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"NavigatorInterface::TargetMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   NavigatorInterface::TargetMessage* tolua_ret = (NavigatorInterface::TargetMessage*)  new NavigatorInterface::TargetMessage();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"NavigatorInterface::TargetMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_NavigatorInterface_TargetMessage_new00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  TargetMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_TargetMessage_new01_local
static int tolua_interfaces_NavigatorInterface_TargetMessage_new01_local(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"NavigatorInterface::TargetMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   NavigatorInterface::TargetMessage* tolua_ret = (NavigatorInterface::TargetMessage*)  new NavigatorInterface::TargetMessage();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"NavigatorInterface::TargetMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_NavigatorInterface_TargetMessage_new00_local(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: delete of class  TargetMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_TargetMessage_delete00
static int tolua_interfaces_NavigatorInterface_TargetMessage_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"NavigatorInterface::TargetMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  NavigatorInterface::TargetMessage* self = (NavigatorInterface::TargetMessage*)  tolua_tousertype(tolua_S,1,0);
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

/* method: x of class  TargetMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_TargetMessage_x00
static int tolua_interfaces_NavigatorInterface_TargetMessage_x00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"NavigatorInterface::TargetMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  NavigatorInterface::TargetMessage* self = (NavigatorInterface::TargetMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'x'",NULL);
#endif
  {
   float tolua_ret = (float)  self->x();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'x'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_x of class  TargetMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_TargetMessage_set_x00
static int tolua_interfaces_NavigatorInterface_TargetMessage_set_x00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"NavigatorInterface::TargetMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  NavigatorInterface::TargetMessage* self = (NavigatorInterface::TargetMessage*)  tolua_tousertype(tolua_S,1,0);
  const float new_x = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_x'",NULL);
#endif
  {
   self->set_x(new_x);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_x'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: y of class  TargetMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_TargetMessage_y00
static int tolua_interfaces_NavigatorInterface_TargetMessage_y00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"NavigatorInterface::TargetMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  NavigatorInterface::TargetMessage* self = (NavigatorInterface::TargetMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'y'",NULL);
#endif
  {
   float tolua_ret = (float)  self->y();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'y'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_y of class  TargetMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_TargetMessage_set_y00
static int tolua_interfaces_NavigatorInterface_TargetMessage_set_y00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"NavigatorInterface::TargetMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  NavigatorInterface::TargetMessage* self = (NavigatorInterface::TargetMessage*)  tolua_tousertype(tolua_S,1,0);
  const float new_y = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_y'",NULL);
#endif
  {
   self->set_y(new_y);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_y'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: orientation of class  TargetMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_TargetMessage_orientation00
static int tolua_interfaces_NavigatorInterface_TargetMessage_orientation00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"NavigatorInterface::TargetMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  NavigatorInterface::TargetMessage* self = (NavigatorInterface::TargetMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'orientation'",NULL);
#endif
  {
   float tolua_ret = (float)  self->orientation();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'orientation'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_orientation of class  TargetMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_TargetMessage_set_orientation00
static int tolua_interfaces_NavigatorInterface_TargetMessage_set_orientation00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"NavigatorInterface::TargetMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  NavigatorInterface::TargetMessage* self = (NavigatorInterface::TargetMessage*)  tolua_tousertype(tolua_S,1,0);
  const float new_orientation = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_orientation'",NULL);
#endif
  {
   self->set_orientation(new_orientation);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_orientation'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  MaxVelocityMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_MaxVelocityMessage_new00
static int tolua_interfaces_NavigatorInterface_MaxVelocityMessage_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"NavigatorInterface::MaxVelocityMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  float ini_velocity = ((float)  tolua_tonumber(tolua_S,2,0));
  {
   NavigatorInterface::MaxVelocityMessage* tolua_ret = (NavigatorInterface::MaxVelocityMessage*)  new NavigatorInterface::MaxVelocityMessage(ini_velocity);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"NavigatorInterface::MaxVelocityMessage");
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

/* method: new_local of class  MaxVelocityMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_MaxVelocityMessage_new00_local
static int tolua_interfaces_NavigatorInterface_MaxVelocityMessage_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"NavigatorInterface::MaxVelocityMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  float ini_velocity = ((float)  tolua_tonumber(tolua_S,2,0));
  {
   NavigatorInterface::MaxVelocityMessage* tolua_ret = (NavigatorInterface::MaxVelocityMessage*)  new NavigatorInterface::MaxVelocityMessage(ini_velocity);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"NavigatorInterface::MaxVelocityMessage");
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

/* method: new of class  MaxVelocityMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_MaxVelocityMessage_new01
static int tolua_interfaces_NavigatorInterface_MaxVelocityMessage_new01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"NavigatorInterface::MaxVelocityMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   NavigatorInterface::MaxVelocityMessage* tolua_ret = (NavigatorInterface::MaxVelocityMessage*)  new NavigatorInterface::MaxVelocityMessage();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"NavigatorInterface::MaxVelocityMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_NavigatorInterface_MaxVelocityMessage_new00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  MaxVelocityMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_MaxVelocityMessage_new01_local
static int tolua_interfaces_NavigatorInterface_MaxVelocityMessage_new01_local(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"NavigatorInterface::MaxVelocityMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   NavigatorInterface::MaxVelocityMessage* tolua_ret = (NavigatorInterface::MaxVelocityMessage*)  new NavigatorInterface::MaxVelocityMessage();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"NavigatorInterface::MaxVelocityMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_NavigatorInterface_MaxVelocityMessage_new00_local(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: delete of class  MaxVelocityMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_MaxVelocityMessage_delete00
static int tolua_interfaces_NavigatorInterface_MaxVelocityMessage_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"NavigatorInterface::MaxVelocityMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  NavigatorInterface::MaxVelocityMessage* self = (NavigatorInterface::MaxVelocityMessage*)  tolua_tousertype(tolua_S,1,0);
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

/* method: velocity of class  MaxVelocityMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_MaxVelocityMessage_velocity00
static int tolua_interfaces_NavigatorInterface_MaxVelocityMessage_velocity00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"NavigatorInterface::MaxVelocityMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  NavigatorInterface::MaxVelocityMessage* self = (NavigatorInterface::MaxVelocityMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'velocity'",NULL);
#endif
  {
   float tolua_ret = (float)  self->velocity();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'velocity'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_velocity of class  MaxVelocityMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_MaxVelocityMessage_set_velocity00
static int tolua_interfaces_NavigatorInterface_MaxVelocityMessage_set_velocity00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"NavigatorInterface::MaxVelocityMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  NavigatorInterface::MaxVelocityMessage* self = (NavigatorInterface::MaxVelocityMessage*)  tolua_tousertype(tolua_S,1,0);
  const float new_velocity = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_velocity'",NULL);
#endif
  {
   self->set_velocity(new_velocity);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_velocity'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  ObstacleMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_ObstacleMessage_new00
static int tolua_interfaces_NavigatorInterface_ObstacleMessage_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"NavigatorInterface::ObstacleMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  float ini_x = ((float)  tolua_tonumber(tolua_S,2,0));
  float ini_y = ((float)  tolua_tonumber(tolua_S,3,0));
  float ini_width = ((float)  tolua_tonumber(tolua_S,4,0));
  {
   NavigatorInterface::ObstacleMessage* tolua_ret = (NavigatorInterface::ObstacleMessage*)  new NavigatorInterface::ObstacleMessage(ini_x,ini_y,ini_width);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"NavigatorInterface::ObstacleMessage");
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

/* method: new_local of class  ObstacleMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_ObstacleMessage_new00_local
static int tolua_interfaces_NavigatorInterface_ObstacleMessage_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"NavigatorInterface::ObstacleMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  float ini_x = ((float)  tolua_tonumber(tolua_S,2,0));
  float ini_y = ((float)  tolua_tonumber(tolua_S,3,0));
  float ini_width = ((float)  tolua_tonumber(tolua_S,4,0));
  {
   NavigatorInterface::ObstacleMessage* tolua_ret = (NavigatorInterface::ObstacleMessage*)  new NavigatorInterface::ObstacleMessage(ini_x,ini_y,ini_width);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"NavigatorInterface::ObstacleMessage");
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

/* method: new of class  ObstacleMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_ObstacleMessage_new01
static int tolua_interfaces_NavigatorInterface_ObstacleMessage_new01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"NavigatorInterface::ObstacleMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   NavigatorInterface::ObstacleMessage* tolua_ret = (NavigatorInterface::ObstacleMessage*)  new NavigatorInterface::ObstacleMessage();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"NavigatorInterface::ObstacleMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_NavigatorInterface_ObstacleMessage_new00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  ObstacleMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_ObstacleMessage_new01_local
static int tolua_interfaces_NavigatorInterface_ObstacleMessage_new01_local(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"NavigatorInterface::ObstacleMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   NavigatorInterface::ObstacleMessage* tolua_ret = (NavigatorInterface::ObstacleMessage*)  new NavigatorInterface::ObstacleMessage();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"NavigatorInterface::ObstacleMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_NavigatorInterface_ObstacleMessage_new00_local(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: delete of class  ObstacleMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_ObstacleMessage_delete00
static int tolua_interfaces_NavigatorInterface_ObstacleMessage_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"NavigatorInterface::ObstacleMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  NavigatorInterface::ObstacleMessage* self = (NavigatorInterface::ObstacleMessage*)  tolua_tousertype(tolua_S,1,0);
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

/* method: x of class  ObstacleMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_ObstacleMessage_x00
static int tolua_interfaces_NavigatorInterface_ObstacleMessage_x00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"NavigatorInterface::ObstacleMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  NavigatorInterface::ObstacleMessage* self = (NavigatorInterface::ObstacleMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'x'",NULL);
#endif
  {
   float tolua_ret = (float)  self->x();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'x'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_x of class  ObstacleMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_ObstacleMessage_set_x00
static int tolua_interfaces_NavigatorInterface_ObstacleMessage_set_x00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"NavigatorInterface::ObstacleMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  NavigatorInterface::ObstacleMessage* self = (NavigatorInterface::ObstacleMessage*)  tolua_tousertype(tolua_S,1,0);
  const float new_x = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_x'",NULL);
#endif
  {
   self->set_x(new_x);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_x'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: y of class  ObstacleMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_ObstacleMessage_y00
static int tolua_interfaces_NavigatorInterface_ObstacleMessage_y00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"NavigatorInterface::ObstacleMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  NavigatorInterface::ObstacleMessage* self = (NavigatorInterface::ObstacleMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'y'",NULL);
#endif
  {
   float tolua_ret = (float)  self->y();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'y'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_y of class  ObstacleMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_ObstacleMessage_set_y00
static int tolua_interfaces_NavigatorInterface_ObstacleMessage_set_y00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"NavigatorInterface::ObstacleMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  NavigatorInterface::ObstacleMessage* self = (NavigatorInterface::ObstacleMessage*)  tolua_tousertype(tolua_S,1,0);
  const float new_y = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_y'",NULL);
#endif
  {
   self->set_y(new_y);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_y'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: width of class  ObstacleMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_ObstacleMessage_width00
static int tolua_interfaces_NavigatorInterface_ObstacleMessage_width00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"NavigatorInterface::ObstacleMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  NavigatorInterface::ObstacleMessage* self = (NavigatorInterface::ObstacleMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'width'",NULL);
#endif
  {
   float tolua_ret = (float)  self->width();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'width'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_width of class  ObstacleMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_ObstacleMessage_set_width00
static int tolua_interfaces_NavigatorInterface_ObstacleMessage_set_width00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"NavigatorInterface::ObstacleMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  NavigatorInterface::ObstacleMessage* self = (NavigatorInterface::ObstacleMessage*)  tolua_tousertype(tolua_S,1,0);
  const float new_width = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_width'",NULL);
#endif
  {
   self->set_width(new_width);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_width'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: foo of class  NavigatorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_foo00
static int tolua_interfaces_NavigatorInterface_foo00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"NavigatorInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  NavigatorInterface* self = (NavigatorInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'foo'",NULL);
#endif
  {
   int tolua_ret = (int)  self->foo();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'foo'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_foo of class  NavigatorInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_NavigatorInterface_set_foo00
static int tolua_interfaces_NavigatorInterface_set_foo00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"NavigatorInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  NavigatorInterface* self = (NavigatorInterface*)  tolua_tousertype(tolua_S,1,0);
  const int new_foo = ((const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_foo'",NULL);
#endif
  {
   self->set_foo(new_foo);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_foo'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* get function: OTHER of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_get_ObjectPositionInterface_unsigned_OTHER
static int tolua_get_ObjectPositionInterface_unsigned_OTHER(lua_State* tolua_S)
{
  tolua_pushnumber(tolua_S,(lua_Number)ObjectPositionInterface::OTHER);
 return 1;
}
#endif //#ifndef TOLUA_DISABLE

/* get function: BALL of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_get_ObjectPositionInterface_unsigned_BALL
static int tolua_get_ObjectPositionInterface_unsigned_BALL(lua_State* tolua_S)
{
  tolua_pushnumber(tolua_S,(lua_Number)ObjectPositionInterface::BALL);
 return 1;
}
#endif //#ifndef TOLUA_DISABLE

/* get function: OPPONENT of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_get_ObjectPositionInterface_unsigned_OPPONENT
static int tolua_get_ObjectPositionInterface_unsigned_OPPONENT(lua_State* tolua_S)
{
  tolua_pushnumber(tolua_S,(lua_Number)ObjectPositionInterface::OPPONENT);
 return 1;
}
#endif //#ifndef TOLUA_DISABLE

/* get function: TEAMMEMBER of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_get_ObjectPositionInterface_unsigned_TEAMMEMBER
static int tolua_get_ObjectPositionInterface_unsigned_TEAMMEMBER(lua_State* tolua_S)
{
  tolua_pushnumber(tolua_S,(lua_Number)ObjectPositionInterface::TEAMMEMBER);
 return 1;
}
#endif //#ifndef TOLUA_DISABLE

/* method: object_type of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_object_type00
static int tolua_interfaces_ObjectPositionInterface_object_type00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'object_type'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->object_type();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'object_type'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_object_type of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_object_type00
static int tolua_interfaces_ObjectPositionInterface_set_object_type00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  unsigned const int new_object_type = ((unsigned const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_object_type'",NULL);
#endif
  {
   self->set_object_type(new_object_type);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_object_type'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: supports_relative of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_supports_relative00
static int tolua_interfaces_ObjectPositionInterface_supports_relative00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'supports_relative'",NULL);
#endif
  {
   char tolua_ret = (char)  self->supports_relative();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'supports_relative'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_supports_relative of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_supports_relative00
static int tolua_interfaces_ObjectPositionInterface_set_supports_relative00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const char new_supports_relative = ((const char)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_supports_relative'",NULL);
#endif
  {
   self->set_supports_relative(new_supports_relative);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_supports_relative'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: supports_global of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_supports_global00
static int tolua_interfaces_ObjectPositionInterface_supports_global00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'supports_global'",NULL);
#endif
  {
   char tolua_ret = (char)  self->supports_global();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'supports_global'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_supports_global of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_supports_global00
static int tolua_interfaces_ObjectPositionInterface_set_supports_global00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const char new_supports_global = ((const char)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_supports_global'",NULL);
#endif
  {
   self->set_supports_global(new_supports_global);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_supports_global'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: has_relative of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_has_relative00
static int tolua_interfaces_ObjectPositionInterface_has_relative00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'has_relative'",NULL);
#endif
  {
   char tolua_ret = (char)  self->has_relative();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'has_relative'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_has_relative of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_has_relative00
static int tolua_interfaces_ObjectPositionInterface_set_has_relative00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const char new_has_relative = ((const char)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_has_relative'",NULL);
#endif
  {
   self->set_has_relative(new_has_relative);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_has_relative'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: has_global of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_has_global00
static int tolua_interfaces_ObjectPositionInterface_has_global00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'has_global'",NULL);
#endif
  {
   char tolua_ret = (char)  self->has_global();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'has_global'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_has_global of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_has_global00
static int tolua_interfaces_ObjectPositionInterface_set_has_global00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const char new_has_global = ((const char)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_has_global'",NULL);
#endif
  {
   self->set_has_global(new_has_global);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_has_global'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: is_visible of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_is_visible00
static int tolua_interfaces_ObjectPositionInterface_is_visible00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'is_visible'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->is_visible();
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'is_visible'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_visible of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_visible00
static int tolua_interfaces_ObjectPositionInterface_set_visible00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isboolean(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const bool new_visible = ((const bool)  tolua_toboolean(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_visible'",NULL);
#endif
  {
   self->set_visible(new_visible);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_visible'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: yaw of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_yaw00
static int tolua_interfaces_ObjectPositionInterface_yaw00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'yaw'",NULL);
#endif
  {
   float tolua_ret = (float)  self->yaw();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'yaw'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_yaw of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_yaw00
static int tolua_interfaces_ObjectPositionInterface_set_yaw00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_yaw = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_yaw'",NULL);
#endif
  {
   self->set_yaw(new_yaw);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_yaw'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: pitch of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_pitch00
static int tolua_interfaces_ObjectPositionInterface_pitch00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'pitch'",NULL);
#endif
  {
   float tolua_ret = (float)  self->pitch();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'pitch'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_pitch of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_pitch00
static int tolua_interfaces_ObjectPositionInterface_set_pitch00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_pitch = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_pitch'",NULL);
#endif
  {
   self->set_pitch(new_pitch);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_pitch'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: distance of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_distance00
static int tolua_interfaces_ObjectPositionInterface_distance00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'distance'",NULL);
#endif
  {
   float tolua_ret = (float)  self->distance();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'distance'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_distance of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_distance00
static int tolua_interfaces_ObjectPositionInterface_set_distance00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_distance = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_distance'",NULL);
#endif
  {
   self->set_distance(new_distance);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_distance'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: dyp_covariance of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_dyp_covariance00
static int tolua_interfaces_ObjectPositionInterface_dyp_covariance00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'dyp_covariance'",NULL);
#endif
  {
   void* tolua_ret = (void*)  self->dyp_covariance();
   tolua_pushuserdata(tolua_S,(void*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'dyp_covariance'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_dyp_covariance of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_dyp_covariance00
static int tolua_interfaces_ObjectPositionInterface_set_dyp_covariance00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_dyp_covariance = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_dyp_covariance'",NULL);
#endif
  {
   self->set_dyp_covariance(&new_dyp_covariance);
   tolua_pushnumber(tolua_S,(lua_Number)new_dyp_covariance);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_dyp_covariance'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: world_x of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_world_x00
static int tolua_interfaces_ObjectPositionInterface_world_x00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'world_x'",NULL);
#endif
  {
   float tolua_ret = (float)  self->world_x();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'world_x'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_world_x of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_world_x00
static int tolua_interfaces_ObjectPositionInterface_set_world_x00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_world_x = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_world_x'",NULL);
#endif
  {
   self->set_world_x(new_world_x);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_world_x'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: world_y of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_world_y00
static int tolua_interfaces_ObjectPositionInterface_world_y00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'world_y'",NULL);
#endif
  {
   float tolua_ret = (float)  self->world_y();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'world_y'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_world_y of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_world_y00
static int tolua_interfaces_ObjectPositionInterface_set_world_y00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_world_y = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_world_y'",NULL);
#endif
  {
   self->set_world_y(new_world_y);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_world_y'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: world_z of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_world_z00
static int tolua_interfaces_ObjectPositionInterface_world_z00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'world_z'",NULL);
#endif
  {
   float tolua_ret = (float)  self->world_z();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'world_z'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_world_z of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_world_z00
static int tolua_interfaces_ObjectPositionInterface_set_world_z00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_world_z = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_world_z'",NULL);
#endif
  {
   self->set_world_z(new_world_z);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_world_z'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: relative_x of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_relative_x00
static int tolua_interfaces_ObjectPositionInterface_relative_x00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'relative_x'",NULL);
#endif
  {
   float tolua_ret = (float)  self->relative_x();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'relative_x'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_relative_x of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_relative_x00
static int tolua_interfaces_ObjectPositionInterface_set_relative_x00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_relative_x = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_relative_x'",NULL);
#endif
  {
   self->set_relative_x(new_relative_x);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_relative_x'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: relative_y of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_relative_y00
static int tolua_interfaces_ObjectPositionInterface_relative_y00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'relative_y'",NULL);
#endif
  {
   float tolua_ret = (float)  self->relative_y();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'relative_y'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_relative_y of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_relative_y00
static int tolua_interfaces_ObjectPositionInterface_set_relative_y00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_relative_y = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_relative_y'",NULL);
#endif
  {
   self->set_relative_y(new_relative_y);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_relative_y'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: relative_z of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_relative_z00
static int tolua_interfaces_ObjectPositionInterface_relative_z00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'relative_z'",NULL);
#endif
  {
   float tolua_ret = (float)  self->relative_z();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'relative_z'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_relative_z of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_relative_z00
static int tolua_interfaces_ObjectPositionInterface_set_relative_z00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_relative_z = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_relative_z'",NULL);
#endif
  {
   self->set_relative_z(new_relative_z);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_relative_z'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: xyz_covariance of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_xyz_covariance00
static int tolua_interfaces_ObjectPositionInterface_xyz_covariance00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'xyz_covariance'",NULL);
#endif
  {
   void* tolua_ret = (void*)  self->xyz_covariance();
   tolua_pushuserdata(tolua_S,(void*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'xyz_covariance'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_xyz_covariance of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_xyz_covariance00
static int tolua_interfaces_ObjectPositionInterface_set_xyz_covariance00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_xyz_covariance = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_xyz_covariance'",NULL);
#endif
  {
   self->set_xyz_covariance(&new_xyz_covariance);
   tolua_pushnumber(tolua_S,(lua_Number)new_xyz_covariance);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_xyz_covariance'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: extent of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_extent00
static int tolua_interfaces_ObjectPositionInterface_extent00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'extent'",NULL);
#endif
  {
   float tolua_ret = (float)  self->extent();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'extent'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_extent of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_extent00
static int tolua_interfaces_ObjectPositionInterface_set_extent00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_extent = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_extent'",NULL);
#endif
  {
   self->set_extent(new_extent);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_extent'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: yaw_velocity of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_yaw_velocity00
static int tolua_interfaces_ObjectPositionInterface_yaw_velocity00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'yaw_velocity'",NULL);
#endif
  {
   float tolua_ret = (float)  self->yaw_velocity();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'yaw_velocity'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_yaw_velocity of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_yaw_velocity00
static int tolua_interfaces_ObjectPositionInterface_set_yaw_velocity00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_yaw_velocity = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_yaw_velocity'",NULL);
#endif
  {
   self->set_yaw_velocity(new_yaw_velocity);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_yaw_velocity'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: pitch_velocity of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_pitch_velocity00
static int tolua_interfaces_ObjectPositionInterface_pitch_velocity00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'pitch_velocity'",NULL);
#endif
  {
   float tolua_ret = (float)  self->pitch_velocity();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'pitch_velocity'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_pitch_velocity of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_pitch_velocity00
static int tolua_interfaces_ObjectPositionInterface_set_pitch_velocity00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_pitch_velocity = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_pitch_velocity'",NULL);
#endif
  {
   self->set_pitch_velocity(new_pitch_velocity);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_pitch_velocity'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: distance_velocity of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_distance_velocity00
static int tolua_interfaces_ObjectPositionInterface_distance_velocity00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'distance_velocity'",NULL);
#endif
  {
   float tolua_ret = (float)  self->distance_velocity();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'distance_velocity'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_distance_velocity of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_distance_velocity00
static int tolua_interfaces_ObjectPositionInterface_set_distance_velocity00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_distance_velocity = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_distance_velocity'",NULL);
#endif
  {
   self->set_distance_velocity(new_distance_velocity);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_distance_velocity'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: dyp_velocity_covariance of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_dyp_velocity_covariance00
static int tolua_interfaces_ObjectPositionInterface_dyp_velocity_covariance00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'dyp_velocity_covariance'",NULL);
#endif
  {
   void* tolua_ret = (void*)  self->dyp_velocity_covariance();
   tolua_pushuserdata(tolua_S,(void*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'dyp_velocity_covariance'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_dyp_velocity_covariance of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_dyp_velocity_covariance00
static int tolua_interfaces_ObjectPositionInterface_set_dyp_velocity_covariance00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_dyp_velocity_covariance = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_dyp_velocity_covariance'",NULL);
#endif
  {
   self->set_dyp_velocity_covariance(&new_dyp_velocity_covariance);
   tolua_pushnumber(tolua_S,(lua_Number)new_dyp_velocity_covariance);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_dyp_velocity_covariance'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: world_x_velocity of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_world_x_velocity00
static int tolua_interfaces_ObjectPositionInterface_world_x_velocity00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'world_x_velocity'",NULL);
#endif
  {
   float tolua_ret = (float)  self->world_x_velocity();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'world_x_velocity'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_world_x_velocity of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_world_x_velocity00
static int tolua_interfaces_ObjectPositionInterface_set_world_x_velocity00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_world_x_velocity = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_world_x_velocity'",NULL);
#endif
  {
   self->set_world_x_velocity(new_world_x_velocity);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_world_x_velocity'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: world_y_velocity of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_world_y_velocity00
static int tolua_interfaces_ObjectPositionInterface_world_y_velocity00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'world_y_velocity'",NULL);
#endif
  {
   float tolua_ret = (float)  self->world_y_velocity();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'world_y_velocity'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_world_y_velocity of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_world_y_velocity00
static int tolua_interfaces_ObjectPositionInterface_set_world_y_velocity00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_world_y_velocity = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_world_y_velocity'",NULL);
#endif
  {
   self->set_world_y_velocity(new_world_y_velocity);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_world_y_velocity'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: world_z_velocity of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_world_z_velocity00
static int tolua_interfaces_ObjectPositionInterface_world_z_velocity00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'world_z_velocity'",NULL);
#endif
  {
   float tolua_ret = (float)  self->world_z_velocity();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'world_z_velocity'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_world_z_velocity of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_world_z_velocity00
static int tolua_interfaces_ObjectPositionInterface_set_world_z_velocity00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_world_z_velocity = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_world_z_velocity'",NULL);
#endif
  {
   self->set_world_z_velocity(new_world_z_velocity);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_world_z_velocity'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: xyz_velocity_covariance of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_xyz_velocity_covariance00
static int tolua_interfaces_ObjectPositionInterface_xyz_velocity_covariance00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'xyz_velocity_covariance'",NULL);
#endif
  {
   void* tolua_ret = (void*)  self->xyz_velocity_covariance();
   tolua_pushuserdata(tolua_S,(void*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'xyz_velocity_covariance'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_xyz_velocity_covariance of class  ObjectPositionInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_ObjectPositionInterface_set_xyz_velocity_covariance00
static int tolua_interfaces_ObjectPositionInterface_set_xyz_velocity_covariance00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"ObjectPositionInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  ObjectPositionInterface* self = (ObjectPositionInterface*)  tolua_tousertype(tolua_S,1,0);
  const float new_xyz_velocity_covariance = ((const float)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_xyz_velocity_covariance'",NULL);
#endif
  {
   self->set_xyz_velocity_covariance(&new_xyz_velocity_covariance);
   tolua_pushnumber(tolua_S,(lua_Number)new_xyz_velocity_covariance);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_xyz_velocity_covariance'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  CallSkillMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_SkillerInterface_CallSkillMessage_new00
static int tolua_interfaces_SkillerInterface_CallSkillMessage_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"SkillerInterface::CallSkillMessage",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  char* ini_skill_string = ((char*)  tolua_tostring(tolua_S,2,0));
  {
   SkillerInterface::CallSkillMessage* tolua_ret = (SkillerInterface::CallSkillMessage*)  new SkillerInterface::CallSkillMessage(ini_skill_string);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"SkillerInterface::CallSkillMessage");
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

/* method: new_local of class  CallSkillMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_SkillerInterface_CallSkillMessage_new00_local
static int tolua_interfaces_SkillerInterface_CallSkillMessage_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"SkillerInterface::CallSkillMessage",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  char* ini_skill_string = ((char*)  tolua_tostring(tolua_S,2,0));
  {
   SkillerInterface::CallSkillMessage* tolua_ret = (SkillerInterface::CallSkillMessage*)  new SkillerInterface::CallSkillMessage(ini_skill_string);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"SkillerInterface::CallSkillMessage");
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

/* method: new of class  CallSkillMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_SkillerInterface_CallSkillMessage_new01
static int tolua_interfaces_SkillerInterface_CallSkillMessage_new01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"SkillerInterface::CallSkillMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   SkillerInterface::CallSkillMessage* tolua_ret = (SkillerInterface::CallSkillMessage*)  new SkillerInterface::CallSkillMessage();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"SkillerInterface::CallSkillMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_SkillerInterface_CallSkillMessage_new00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  CallSkillMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_SkillerInterface_CallSkillMessage_new01_local
static int tolua_interfaces_SkillerInterface_CallSkillMessage_new01_local(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"SkillerInterface::CallSkillMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   SkillerInterface::CallSkillMessage* tolua_ret = (SkillerInterface::CallSkillMessage*)  new SkillerInterface::CallSkillMessage();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"SkillerInterface::CallSkillMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_SkillerInterface_CallSkillMessage_new00_local(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: delete of class  CallSkillMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_SkillerInterface_CallSkillMessage_delete00
static int tolua_interfaces_SkillerInterface_CallSkillMessage_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"SkillerInterface::CallSkillMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  SkillerInterface::CallSkillMessage* self = (SkillerInterface::CallSkillMessage*)  tolua_tousertype(tolua_S,1,0);
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

/* method: skill_string of class  CallSkillMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_SkillerInterface_CallSkillMessage_skill_string00
static int tolua_interfaces_SkillerInterface_CallSkillMessage_skill_string00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"SkillerInterface::CallSkillMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  SkillerInterface::CallSkillMessage* self = (SkillerInterface::CallSkillMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'skill_string'",NULL);
#endif
  {
   char* tolua_ret = (char*)  self->skill_string();
   tolua_pushstring(tolua_S,(const char*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'skill_string'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_skill_string of class  CallSkillMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_SkillerInterface_CallSkillMessage_set_skill_string00
static int tolua_interfaces_SkillerInterface_CallSkillMessage_set_skill_string00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"SkillerInterface::CallSkillMessage",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  SkillerInterface::CallSkillMessage* self = (SkillerInterface::CallSkillMessage*)  tolua_tousertype(tolua_S,1,0);
  const char* new_skill_string = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_skill_string'",NULL);
#endif
  {
   self->set_skill_string(new_skill_string);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_skill_string'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  RestartInterpreterMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_SkillerInterface_RestartInterpreterMessage_new00
static int tolua_interfaces_SkillerInterface_RestartInterpreterMessage_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"SkillerInterface::RestartInterpreterMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   SkillerInterface::RestartInterpreterMessage* tolua_ret = (SkillerInterface::RestartInterpreterMessage*)  new SkillerInterface::RestartInterpreterMessage();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"SkillerInterface::RestartInterpreterMessage");
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

/* method: new_local of class  RestartInterpreterMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_SkillerInterface_RestartInterpreterMessage_new00_local
static int tolua_interfaces_SkillerInterface_RestartInterpreterMessage_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"SkillerInterface::RestartInterpreterMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   SkillerInterface::RestartInterpreterMessage* tolua_ret = (SkillerInterface::RestartInterpreterMessage*)  new SkillerInterface::RestartInterpreterMessage();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"SkillerInterface::RestartInterpreterMessage");
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

/* method: delete of class  RestartInterpreterMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_SkillerInterface_RestartInterpreterMessage_delete00
static int tolua_interfaces_SkillerInterface_RestartInterpreterMessage_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"SkillerInterface::RestartInterpreterMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  SkillerInterface::RestartInterpreterMessage* self = (SkillerInterface::RestartInterpreterMessage*)  tolua_tousertype(tolua_S,1,0);
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

/* method: skill_string of class  SkillerInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_SkillerInterface_skill_string00
static int tolua_interfaces_SkillerInterface_skill_string00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"SkillerInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  SkillerInterface* self = (SkillerInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'skill_string'",NULL);
#endif
  {
   char* tolua_ret = (char*)  self->skill_string();
   tolua_pushstring(tolua_S,(const char*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'skill_string'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_skill_string of class  SkillerInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_SkillerInterface_set_skill_string00
static int tolua_interfaces_SkillerInterface_set_skill_string00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"SkillerInterface",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  SkillerInterface* self = (SkillerInterface*)  tolua_tousertype(tolua_S,1,0);
  const char* new_skill_string = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_skill_string'",NULL);
#endif
  {
   self->set_skill_string(new_skill_string);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_skill_string'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* get function: TEST_CONSTANT of class  TestInterface */
#ifndef TOLUA_DISABLE_tolua_get_TestInterface_TEST_CONSTANT
static int tolua_get_TestInterface_TEST_CONSTANT(lua_State* tolua_S)
{
  tolua_pushnumber(tolua_S,(lua_Number)TestInterface::TEST_CONSTANT);
 return 1;
}
#endif //#ifndef TOLUA_DISABLE

/* get function: TEST_FLOAT_CONSTANT of class  TestInterface */
#ifndef TOLUA_DISABLE_tolua_get_TestInterface_TEST_FLOAT_CONSTANT
static int tolua_get_TestInterface_TEST_FLOAT_CONSTANT(lua_State* tolua_S)
{
  tolua_pushnumber(tolua_S,(lua_Number)TestInterface::TEST_FLOAT_CONSTANT);
 return 1;
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  SetTestIntMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_SetTestIntMessage_new00
static int tolua_interfaces_TestInterface_SetTestIntMessage_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"TestInterface::SetTestIntMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  int ini_test_int = ((int)  tolua_tonumber(tolua_S,2,0));
  {
   TestInterface::SetTestIntMessage* tolua_ret = (TestInterface::SetTestIntMessage*)  new TestInterface::SetTestIntMessage(ini_test_int);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"TestInterface::SetTestIntMessage");
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

/* method: new_local of class  SetTestIntMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_SetTestIntMessage_new00_local
static int tolua_interfaces_TestInterface_SetTestIntMessage_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"TestInterface::SetTestIntMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  int ini_test_int = ((int)  tolua_tonumber(tolua_S,2,0));
  {
   TestInterface::SetTestIntMessage* tolua_ret = (TestInterface::SetTestIntMessage*)  new TestInterface::SetTestIntMessage(ini_test_int);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"TestInterface::SetTestIntMessage");
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

/* method: new of class  SetTestIntMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_SetTestIntMessage_new01
static int tolua_interfaces_TestInterface_SetTestIntMessage_new01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"TestInterface::SetTestIntMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   TestInterface::SetTestIntMessage* tolua_ret = (TestInterface::SetTestIntMessage*)  new TestInterface::SetTestIntMessage();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"TestInterface::SetTestIntMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_TestInterface_SetTestIntMessage_new00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  SetTestIntMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_SetTestIntMessage_new01_local
static int tolua_interfaces_TestInterface_SetTestIntMessage_new01_local(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"TestInterface::SetTestIntMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   TestInterface::SetTestIntMessage* tolua_ret = (TestInterface::SetTestIntMessage*)  new TestInterface::SetTestIntMessage();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"TestInterface::SetTestIntMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_TestInterface_SetTestIntMessage_new00_local(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: delete of class  SetTestIntMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_SetTestIntMessage_delete00
static int tolua_interfaces_TestInterface_SetTestIntMessage_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface::SetTestIntMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface::SetTestIntMessage* self = (TestInterface::SetTestIntMessage*)  tolua_tousertype(tolua_S,1,0);
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

/* method: test_int of class  SetTestIntMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_SetTestIntMessage_test_int00
static int tolua_interfaces_TestInterface_SetTestIntMessage_test_int00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface::SetTestIntMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface::SetTestIntMessage* self = (TestInterface::SetTestIntMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'test_int'",NULL);
#endif
  {
   int tolua_ret = (int)  self->test_int();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'test_int'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_test_int of class  SetTestIntMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_SetTestIntMessage_set_test_int00
static int tolua_interfaces_TestInterface_SetTestIntMessage_set_test_int00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface::SetTestIntMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface::SetTestIntMessage* self = (TestInterface::SetTestIntMessage*)  tolua_tousertype(tolua_S,1,0);
  const int new_test_int = ((const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_test_int'",NULL);
#endif
  {
   self->set_test_int(new_test_int);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_test_int'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  SetTestStringMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_SetTestStringMessage_new00
static int tolua_interfaces_TestInterface_SetTestStringMessage_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"TestInterface::SetTestStringMessage",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  char* ini_test_string = ((char*)  tolua_tostring(tolua_S,2,0));
  {
   TestInterface::SetTestStringMessage* tolua_ret = (TestInterface::SetTestStringMessage*)  new TestInterface::SetTestStringMessage(ini_test_string);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"TestInterface::SetTestStringMessage");
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

/* method: new_local of class  SetTestStringMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_SetTestStringMessage_new00_local
static int tolua_interfaces_TestInterface_SetTestStringMessage_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"TestInterface::SetTestStringMessage",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  char* ini_test_string = ((char*)  tolua_tostring(tolua_S,2,0));
  {
   TestInterface::SetTestStringMessage* tolua_ret = (TestInterface::SetTestStringMessage*)  new TestInterface::SetTestStringMessage(ini_test_string);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"TestInterface::SetTestStringMessage");
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

/* method: new of class  SetTestStringMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_SetTestStringMessage_new01
static int tolua_interfaces_TestInterface_SetTestStringMessage_new01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"TestInterface::SetTestStringMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   TestInterface::SetTestStringMessage* tolua_ret = (TestInterface::SetTestStringMessage*)  new TestInterface::SetTestStringMessage();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"TestInterface::SetTestStringMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_TestInterface_SetTestStringMessage_new00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  SetTestStringMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_SetTestStringMessage_new01_local
static int tolua_interfaces_TestInterface_SetTestStringMessage_new01_local(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"TestInterface::SetTestStringMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   TestInterface::SetTestStringMessage* tolua_ret = (TestInterface::SetTestStringMessage*)  new TestInterface::SetTestStringMessage();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"TestInterface::SetTestStringMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_TestInterface_SetTestStringMessage_new00_local(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: delete of class  SetTestStringMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_SetTestStringMessage_delete00
static int tolua_interfaces_TestInterface_SetTestStringMessage_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface::SetTestStringMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface::SetTestStringMessage* self = (TestInterface::SetTestStringMessage*)  tolua_tousertype(tolua_S,1,0);
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

/* method: test_string of class  SetTestStringMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_SetTestStringMessage_test_string00
static int tolua_interfaces_TestInterface_SetTestStringMessage_test_string00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface::SetTestStringMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface::SetTestStringMessage* self = (TestInterface::SetTestStringMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'test_string'",NULL);
#endif
  {
   char* tolua_ret = (char*)  self->test_string();
   tolua_pushstring(tolua_S,(const char*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'test_string'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_test_string of class  SetTestStringMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_SetTestStringMessage_set_test_string00
static int tolua_interfaces_TestInterface_SetTestStringMessage_set_test_string00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface::SetTestStringMessage",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface::SetTestStringMessage* self = (TestInterface::SetTestStringMessage*)  tolua_tousertype(tolua_S,1,0);
  const char* new_test_string = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_test_string'",NULL);
#endif
  {
   self->set_test_string(new_test_string);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_test_string'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  CalculateMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_CalculateMessage_new00
static int tolua_interfaces_TestInterface_CalculateMessage_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"TestInterface::CalculateMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  int ini_summand = ((int)  tolua_tonumber(tolua_S,2,0));
  int ini_addend = ((int)  tolua_tonumber(tolua_S,3,0));
  {
   TestInterface::CalculateMessage* tolua_ret = (TestInterface::CalculateMessage*)  new TestInterface::CalculateMessage(ini_summand,ini_addend);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"TestInterface::CalculateMessage");
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

/* method: new_local of class  CalculateMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_CalculateMessage_new00_local
static int tolua_interfaces_TestInterface_CalculateMessage_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"TestInterface::CalculateMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  int ini_summand = ((int)  tolua_tonumber(tolua_S,2,0));
  int ini_addend = ((int)  tolua_tonumber(tolua_S,3,0));
  {
   TestInterface::CalculateMessage* tolua_ret = (TestInterface::CalculateMessage*)  new TestInterface::CalculateMessage(ini_summand,ini_addend);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"TestInterface::CalculateMessage");
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

/* method: new of class  CalculateMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_CalculateMessage_new01
static int tolua_interfaces_TestInterface_CalculateMessage_new01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"TestInterface::CalculateMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   TestInterface::CalculateMessage* tolua_ret = (TestInterface::CalculateMessage*)  new TestInterface::CalculateMessage();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"TestInterface::CalculateMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_TestInterface_CalculateMessage_new00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  CalculateMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_CalculateMessage_new01_local
static int tolua_interfaces_TestInterface_CalculateMessage_new01_local(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"TestInterface::CalculateMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  {
   TestInterface::CalculateMessage* tolua_ret = (TestInterface::CalculateMessage*)  new TestInterface::CalculateMessage();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"TestInterface::CalculateMessage");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interfaces_TestInterface_CalculateMessage_new00_local(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: delete of class  CalculateMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_CalculateMessage_delete00
static int tolua_interfaces_TestInterface_CalculateMessage_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface::CalculateMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface::CalculateMessage* self = (TestInterface::CalculateMessage*)  tolua_tousertype(tolua_S,1,0);
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

/* method: summand of class  CalculateMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_CalculateMessage_summand00
static int tolua_interfaces_TestInterface_CalculateMessage_summand00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface::CalculateMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface::CalculateMessage* self = (TestInterface::CalculateMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'summand'",NULL);
#endif
  {
   int tolua_ret = (int)  self->summand();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'summand'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_summand of class  CalculateMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_CalculateMessage_set_summand00
static int tolua_interfaces_TestInterface_CalculateMessage_set_summand00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface::CalculateMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface::CalculateMessage* self = (TestInterface::CalculateMessage*)  tolua_tousertype(tolua_S,1,0);
  const int new_summand = ((const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_summand'",NULL);
#endif
  {
   self->set_summand(new_summand);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_summand'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: addend of class  CalculateMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_CalculateMessage_addend00
static int tolua_interfaces_TestInterface_CalculateMessage_addend00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface::CalculateMessage",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface::CalculateMessage* self = (TestInterface::CalculateMessage*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'addend'",NULL);
#endif
  {
   int tolua_ret = (int)  self->addend();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'addend'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_addend of class  CalculateMessage */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_CalculateMessage_set_addend00
static int tolua_interfaces_TestInterface_CalculateMessage_set_addend00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface::CalculateMessage",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface::CalculateMessage* self = (TestInterface::CalculateMessage*)  tolua_tousertype(tolua_S,1,0);
  const int new_addend = ((const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_addend'",NULL);
#endif
  {
   self->set_addend(new_addend);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_addend'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: is_test_bool of class  TestInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_is_test_bool00
static int tolua_interfaces_TestInterface_is_test_bool00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface* self = (TestInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'is_test_bool'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->is_test_bool();
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'is_test_bool'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_test_bool of class  TestInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_set_test_bool00
static int tolua_interfaces_TestInterface_set_test_bool00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface",0,&tolua_err) ||
     !tolua_isboolean(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface* self = (TestInterface*)  tolua_tousertype(tolua_S,1,0);
  const bool new_test_bool = ((const bool)  tolua_toboolean(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_test_bool'",NULL);
#endif
  {
   self->set_test_bool(new_test_bool);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_test_bool'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: test_int of class  TestInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_test_int00
static int tolua_interfaces_TestInterface_test_int00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface* self = (TestInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'test_int'",NULL);
#endif
  {
   int tolua_ret = (int)  self->test_int();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'test_int'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_test_int of class  TestInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_set_test_int00
static int tolua_interfaces_TestInterface_set_test_int00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface* self = (TestInterface*)  tolua_tousertype(tolua_S,1,0);
  const int new_test_int = ((const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_test_int'",NULL);
#endif
  {
   self->set_test_int(new_test_int);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_test_int'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: _flags of class  TestInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface__flags00
static int tolua_interfaces_TestInterface__flags00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface* self = (TestInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function '_flags'",NULL);
#endif
  {
   char tolua_ret = (char)  self->_flags();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function '_flags'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set__flags of class  TestInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_set__flags00
static int tolua_interfaces_TestInterface_set__flags00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface* self = (TestInterface*)  tolua_tousertype(tolua_S,1,0);
  const char new__flags = ((const char)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set__flags'",NULL);
#endif
  {
   self->set__flags(new__flags);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set__flags'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: test_string of class  TestInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_test_string00
static int tolua_interfaces_TestInterface_test_string00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface* self = (TestInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'test_string'",NULL);
#endif
  {
   char* tolua_ret = (char*)  self->test_string();
   tolua_pushstring(tolua_S,(const char*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'test_string'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_test_string of class  TestInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_set_test_string00
static int tolua_interfaces_TestInterface_set_test_string00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface* self = (TestInterface*)  tolua_tousertype(tolua_S,1,0);
  const char* new_test_string = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_test_string'",NULL);
#endif
  {
   self->set_test_string(new_test_string);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_test_string'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: result of class  TestInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_result00
static int tolua_interfaces_TestInterface_result00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface* self = (TestInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'result'",NULL);
#endif
  {
   int tolua_ret = (int)  self->result();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'result'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_result of class  TestInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_set_result00
static int tolua_interfaces_TestInterface_set_result00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface* self = (TestInterface*)  tolua_tousertype(tolua_S,1,0);
  const int new_result = ((const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_result'",NULL);
#endif
  {
   self->set_result(new_result);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_result'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: test_uint of class  TestInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_test_uint00
static int tolua_interfaces_TestInterface_test_uint00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface* self = (TestInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'test_uint'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->test_uint();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'test_uint'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_test_uint of class  TestInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_set_test_uint00
static int tolua_interfaces_TestInterface_set_test_uint00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface* self = (TestInterface*)  tolua_tousertype(tolua_S,1,0);
  unsigned const int new_test_uint = ((unsigned const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_test_uint'",NULL);
#endif
  {
   self->set_test_uint(new_test_uint);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_test_uint'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: test_ulint of class  TestInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_test_ulint00
static int tolua_interfaces_TestInterface_test_ulint00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface* self = (TestInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'test_ulint'",NULL);
#endif
  {
   unsigned long int tolua_ret = (unsigned long int)  self->test_ulint();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'test_ulint'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_test_ulint of class  TestInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_set_test_ulint00
static int tolua_interfaces_TestInterface_set_test_ulint00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface* self = (TestInterface*)  tolua_tousertype(tolua_S,1,0);
  unsigned long const int new_test_ulint = ((unsigned long const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_test_ulint'",NULL);
#endif
  {
   self->set_test_ulint(new_test_ulint);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_test_ulint'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: test_lint of class  TestInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_test_lint00
static int tolua_interfaces_TestInterface_test_lint00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface* self = (TestInterface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'test_lint'",NULL);
#endif
  {
   long int tolua_ret = (long int)  self->test_lint();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'test_lint'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_test_lint of class  TestInterface */
#ifndef TOLUA_DISABLE_tolua_interfaces_TestInterface_set_test_lint00
static int tolua_interfaces_TestInterface_set_test_lint00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"TestInterface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  TestInterface* self = (TestInterface*)  tolua_tousertype(tolua_S,1,0);
  long const int new_test_lint = ((long const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_test_lint'",NULL);
#endif
  {
   self->set_test_lint(new_test_lint);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_test_lint'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* Open function */
TOLUA_API int tolua_interfaces_open (lua_State* tolua_S)
{
 tolua_open(tolua_S);
 tolua_reg_types(tolua_S);
 tolua_module(tolua_S,NULL,0);
 tolua_beginmodule(tolua_S,NULL);
  tolua_cclass(tolua_S,"BatteryInterface","BatteryInterface","Interface",NULL);
  tolua_beginmodule(tolua_S,"BatteryInterface");
   #ifdef __cplusplus
   tolua_cclass(tolua_S,"PushButtonMessage","BatteryInterface::PushButtonMessage","Message",tolua_collect_BatteryInterface__PushButtonMessage);
   #else
   tolua_cclass(tolua_S,"PushButtonMessage","BatteryInterface::PushButtonMessage","Message",NULL);
   #endif
   tolua_beginmodule(tolua_S,"PushButtonMessage");
    tolua_function(tolua_S,"new",tolua_interfaces_BatteryInterface_PushButtonMessage_new00);
    tolua_function(tolua_S,"new_local",tolua_interfaces_BatteryInterface_PushButtonMessage_new00_local);
    tolua_function(tolua_S,".call",tolua_interfaces_BatteryInterface_PushButtonMessage_new00_local);
    tolua_function(tolua_S,"delete",tolua_interfaces_BatteryInterface_PushButtonMessage_delete00);
   tolua_endmodule(tolua_S);
   #ifdef __cplusplus
   tolua_cclass(tolua_S,"SleepMessage","BatteryInterface::SleepMessage","Message",tolua_collect_BatteryInterface__SleepMessage);
   #else
   tolua_cclass(tolua_S,"SleepMessage","BatteryInterface::SleepMessage","Message",NULL);
   #endif
   tolua_beginmodule(tolua_S,"SleepMessage");
    tolua_function(tolua_S,"new",tolua_interfaces_BatteryInterface_SleepMessage_new00);
    tolua_function(tolua_S,"new_local",tolua_interfaces_BatteryInterface_SleepMessage_new00_local);
    tolua_function(tolua_S,".call",tolua_interfaces_BatteryInterface_SleepMessage_new00_local);
    tolua_function(tolua_S,"delete",tolua_interfaces_BatteryInterface_SleepMessage_delete00);
   tolua_endmodule(tolua_S);
   tolua_function(tolua_S,"current",tolua_interfaces_BatteryInterface_current00);
   tolua_function(tolua_S,"set_current",tolua_interfaces_BatteryInterface_set_current00);
   tolua_function(tolua_S,"voltage",tolua_interfaces_BatteryInterface_voltage00);
   tolua_function(tolua_S,"set_voltage",tolua_interfaces_BatteryInterface_set_voltage00);
   tolua_function(tolua_S,"temperature",tolua_interfaces_BatteryInterface_temperature00);
   tolua_function(tolua_S,"set_temperature",tolua_interfaces_BatteryInterface_set_temperature00);
  tolua_endmodule(tolua_S);
  tolua_cclass(tolua_S,"KickerInterface","KickerInterface","Interface",NULL);
  tolua_beginmodule(tolua_S,"KickerInterface");
   tolua_constant(tolua_S,"GUIDE_BALL_LEFT",KickerInterface::GUIDE_BALL_LEFT);
   tolua_constant(tolua_S,"GUIDE_BALL_RIGHT",KickerInterface::GUIDE_BALL_RIGHT);
   #ifdef __cplusplus
   tolua_cclass(tolua_S,"KickMessage","KickerInterface::KickMessage","Message",tolua_collect_KickerInterface__KickMessage);
   #else
   tolua_cclass(tolua_S,"KickMessage","KickerInterface::KickMessage","Message",NULL);
   #endif
   tolua_beginmodule(tolua_S,"KickMessage");
    tolua_function(tolua_S,"new",tolua_interfaces_KickerInterface_KickMessage_new00);
    tolua_function(tolua_S,"new_local",tolua_interfaces_KickerInterface_KickMessage_new00_local);
    tolua_function(tolua_S,".call",tolua_interfaces_KickerInterface_KickMessage_new00_local);
    tolua_function(tolua_S,"new",tolua_interfaces_KickerInterface_KickMessage_new01);
    tolua_function(tolua_S,"new_local",tolua_interfaces_KickerInterface_KickMessage_new01_local);
    tolua_function(tolua_S,".call",tolua_interfaces_KickerInterface_KickMessage_new01_local);
    tolua_function(tolua_S,"delete",tolua_interfaces_KickerInterface_KickMessage_delete00);
    tolua_function(tolua_S,"is_left",tolua_interfaces_KickerInterface_KickMessage_is_left00);
    tolua_function(tolua_S,"set_left",tolua_interfaces_KickerInterface_KickMessage_set_left00);
    tolua_function(tolua_S,"is_center",tolua_interfaces_KickerInterface_KickMessage_is_center00);
    tolua_function(tolua_S,"set_center",tolua_interfaces_KickerInterface_KickMessage_set_center00);
    tolua_function(tolua_S,"is_right",tolua_interfaces_KickerInterface_KickMessage_is_right00);
    tolua_function(tolua_S,"set_right",tolua_interfaces_KickerInterface_KickMessage_set_right00);
    tolua_function(tolua_S,"intensity",tolua_interfaces_KickerInterface_KickMessage_intensity00);
    tolua_function(tolua_S,"set_intensity",tolua_interfaces_KickerInterface_KickMessage_set_intensity00);
   tolua_endmodule(tolua_S);
   #ifdef __cplusplus
   tolua_cclass(tolua_S,"ResetCounterMessage","KickerInterface::ResetCounterMessage","Message",tolua_collect_KickerInterface__ResetCounterMessage);
   #else
   tolua_cclass(tolua_S,"ResetCounterMessage","KickerInterface::ResetCounterMessage","Message",NULL);
   #endif
   tolua_beginmodule(tolua_S,"ResetCounterMessage");
    tolua_function(tolua_S,"new",tolua_interfaces_KickerInterface_ResetCounterMessage_new00);
    tolua_function(tolua_S,"new_local",tolua_interfaces_KickerInterface_ResetCounterMessage_new00_local);
    tolua_function(tolua_S,".call",tolua_interfaces_KickerInterface_ResetCounterMessage_new00_local);
    tolua_function(tolua_S,"delete",tolua_interfaces_KickerInterface_ResetCounterMessage_delete00);
   tolua_endmodule(tolua_S);
   #ifdef __cplusplus
   tolua_cclass(tolua_S,"GuideBallMessage","KickerInterface::GuideBallMessage","Message",tolua_collect_KickerInterface__GuideBallMessage);
   #else
   tolua_cclass(tolua_S,"GuideBallMessage","KickerInterface::GuideBallMessage","Message",NULL);
   #endif
   tolua_beginmodule(tolua_S,"GuideBallMessage");
    tolua_function(tolua_S,"new",tolua_interfaces_KickerInterface_GuideBallMessage_new00);
    tolua_function(tolua_S,"new_local",tolua_interfaces_KickerInterface_GuideBallMessage_new00_local);
    tolua_function(tolua_S,".call",tolua_interfaces_KickerInterface_GuideBallMessage_new00_local);
    tolua_function(tolua_S,"new",tolua_interfaces_KickerInterface_GuideBallMessage_new01);
    tolua_function(tolua_S,"new_local",tolua_interfaces_KickerInterface_GuideBallMessage_new01_local);
    tolua_function(tolua_S,".call",tolua_interfaces_KickerInterface_GuideBallMessage_new01_local);
    tolua_function(tolua_S,"delete",tolua_interfaces_KickerInterface_GuideBallMessage_delete00);
    tolua_function(tolua_S,"guide_ball_side",tolua_interfaces_KickerInterface_GuideBallMessage_guide_ball_side00);
    tolua_function(tolua_S,"set_guide_ball_side",tolua_interfaces_KickerInterface_GuideBallMessage_set_guide_ball_side00);
   tolua_endmodule(tolua_S);
   tolua_function(tolua_S,"num_kicks_left",tolua_interfaces_KickerInterface_num_kicks_left00);
   tolua_function(tolua_S,"set_num_kicks_left",tolua_interfaces_KickerInterface_set_num_kicks_left00);
   tolua_function(tolua_S,"num_kicks_center",tolua_interfaces_KickerInterface_num_kicks_center00);
   tolua_function(tolua_S,"set_num_kicks_center",tolua_interfaces_KickerInterface_set_num_kicks_center00);
   tolua_function(tolua_S,"num_kicks_right",tolua_interfaces_KickerInterface_num_kicks_right00);
   tolua_function(tolua_S,"set_num_kicks_right",tolua_interfaces_KickerInterface_set_num_kicks_right00);
   tolua_function(tolua_S,"guide_ball_side",tolua_interfaces_KickerInterface_guide_ball_side00);
   tolua_function(tolua_S,"set_guide_ball_side",tolua_interfaces_KickerInterface_set_guide_ball_side00);
   tolua_function(tolua_S,"current_intensity",tolua_interfaces_KickerInterface_current_intensity00);
   tolua_function(tolua_S,"set_current_intensity",tolua_interfaces_KickerInterface_set_current_intensity00);
  tolua_endmodule(tolua_S);
  tolua_cclass(tolua_S,"MotorInterface","MotorInterface","Interface",NULL);
  tolua_beginmodule(tolua_S,"MotorInterface");
   tolua_variable(tolua_S,"MOTOR_ENABLED",tolua_get_MotorInterface_unsigned_MOTOR_ENABLED,NULL);
   tolua_variable(tolua_S,"MOTOR_DISABLED",tolua_get_MotorInterface_unsigned_MOTOR_DISABLED,NULL);
   tolua_variable(tolua_S,"DRIVE_MODE_RPM",tolua_get_MotorInterface_unsigned_DRIVE_MODE_RPM,NULL);
   tolua_variable(tolua_S,"DRIVE_MODE_TRANS",tolua_get_MotorInterface_unsigned_DRIVE_MODE_TRANS,NULL);
   tolua_variable(tolua_S,"DRIVE_MODE_ROT",tolua_get_MotorInterface_unsigned_DRIVE_MODE_ROT,NULL);
   tolua_variable(tolua_S,"DRIVE_MODE_TRANS_ROT",tolua_get_MotorInterface_unsigned_DRIVE_MODE_TRANS_ROT,NULL);
   tolua_variable(tolua_S,"DRIVE_MODE_ORBIT",tolua_get_MotorInterface_unsigned_DRIVE_MODE_ORBIT,NULL);
   tolua_variable(tolua_S,"DRIVE_MODE_LINE_TRANS_ROT",tolua_get_MotorInterface_unsigned_DRIVE_MODE_LINE_TRANS_ROT,NULL);
   #ifdef __cplusplus
   tolua_cclass(tolua_S,"SetMotorStateMessage","MotorInterface::SetMotorStateMessage","Message",tolua_collect_MotorInterface__SetMotorStateMessage);
   #else
   tolua_cclass(tolua_S,"SetMotorStateMessage","MotorInterface::SetMotorStateMessage","Message",NULL);
   #endif
   tolua_beginmodule(tolua_S,"SetMotorStateMessage");
    tolua_function(tolua_S,"new",tolua_interfaces_MotorInterface_SetMotorStateMessage_new00);
    tolua_function(tolua_S,"new_local",tolua_interfaces_MotorInterface_SetMotorStateMessage_new00_local);
    tolua_function(tolua_S,".call",tolua_interfaces_MotorInterface_SetMotorStateMessage_new00_local);
    tolua_function(tolua_S,"new",tolua_interfaces_MotorInterface_SetMotorStateMessage_new01);
    tolua_function(tolua_S,"new_local",tolua_interfaces_MotorInterface_SetMotorStateMessage_new01_local);
    tolua_function(tolua_S,".call",tolua_interfaces_MotorInterface_SetMotorStateMessage_new01_local);
    tolua_function(tolua_S,"delete",tolua_interfaces_MotorInterface_SetMotorStateMessage_delete00);
    tolua_function(tolua_S,"motor_state",tolua_interfaces_MotorInterface_SetMotorStateMessage_motor_state00);
    tolua_function(tolua_S,"set_motor_state",tolua_interfaces_MotorInterface_SetMotorStateMessage_set_motor_state00);
   tolua_endmodule(tolua_S);
   #ifdef __cplusplus
   tolua_cclass(tolua_S,"AcquireControlMessage","MotorInterface::AcquireControlMessage","Message",tolua_collect_MotorInterface__AcquireControlMessage);
   #else
   tolua_cclass(tolua_S,"AcquireControlMessage","MotorInterface::AcquireControlMessage","Message",NULL);
   #endif
   tolua_beginmodule(tolua_S,"AcquireControlMessage");
    tolua_function(tolua_S,"new",tolua_interfaces_MotorInterface_AcquireControlMessage_new00);
    tolua_function(tolua_S,"new_local",tolua_interfaces_MotorInterface_AcquireControlMessage_new00_local);
    tolua_function(tolua_S,".call",tolua_interfaces_MotorInterface_AcquireControlMessage_new00_local);
    tolua_function(tolua_S,"new",tolua_interfaces_MotorInterface_AcquireControlMessage_new01);
    tolua_function(tolua_S,"new_local",tolua_interfaces_MotorInterface_AcquireControlMessage_new01_local);
    tolua_function(tolua_S,".call",tolua_interfaces_MotorInterface_AcquireControlMessage_new01_local);
    tolua_function(tolua_S,"delete",tolua_interfaces_MotorInterface_AcquireControlMessage_delete00);
    tolua_function(tolua_S,"thread_id",tolua_interfaces_MotorInterface_AcquireControlMessage_thread_id00);
    tolua_function(tolua_S,"set_thread_id",tolua_interfaces_MotorInterface_AcquireControlMessage_set_thread_id00);
    tolua_function(tolua_S,"thread_name",tolua_interfaces_MotorInterface_AcquireControlMessage_thread_name00);
    tolua_function(tolua_S,"set_thread_name",tolua_interfaces_MotorInterface_AcquireControlMessage_set_thread_name00);
   tolua_endmodule(tolua_S);
   #ifdef __cplusplus
   tolua_cclass(tolua_S,"ResetOdometryMessage","MotorInterface::ResetOdometryMessage","Message",tolua_collect_MotorInterface__ResetOdometryMessage);
   #else
   tolua_cclass(tolua_S,"ResetOdometryMessage","MotorInterface::ResetOdometryMessage","Message",NULL);
   #endif
   tolua_beginmodule(tolua_S,"ResetOdometryMessage");
    tolua_function(tolua_S,"new",tolua_interfaces_MotorInterface_ResetOdometryMessage_new00);
    tolua_function(tolua_S,"new_local",tolua_interfaces_MotorInterface_ResetOdometryMessage_new00_local);
    tolua_function(tolua_S,".call",tolua_interfaces_MotorInterface_ResetOdometryMessage_new00_local);
    tolua_function(tolua_S,"delete",tolua_interfaces_MotorInterface_ResetOdometryMessage_delete00);
   tolua_endmodule(tolua_S);
   #ifdef __cplusplus
   tolua_cclass(tolua_S,"DriveRPMMessage","MotorInterface::DriveRPMMessage","Message",tolua_collect_MotorInterface__DriveRPMMessage);
   #else
   tolua_cclass(tolua_S,"DriveRPMMessage","MotorInterface::DriveRPMMessage","Message",NULL);
   #endif
   tolua_beginmodule(tolua_S,"DriveRPMMessage");
    tolua_function(tolua_S,"new",tolua_interfaces_MotorInterface_DriveRPMMessage_new00);
    tolua_function(tolua_S,"new_local",tolua_interfaces_MotorInterface_DriveRPMMessage_new00_local);
    tolua_function(tolua_S,".call",tolua_interfaces_MotorInterface_DriveRPMMessage_new00_local);
    tolua_function(tolua_S,"new",tolua_interfaces_MotorInterface_DriveRPMMessage_new01);
    tolua_function(tolua_S,"new_local",tolua_interfaces_MotorInterface_DriveRPMMessage_new01_local);
    tolua_function(tolua_S,".call",tolua_interfaces_MotorInterface_DriveRPMMessage_new01_local);
    tolua_function(tolua_S,"delete",tolua_interfaces_MotorInterface_DriveRPMMessage_delete00);
    tolua_function(tolua_S,"front_right",tolua_interfaces_MotorInterface_DriveRPMMessage_front_right00);
    tolua_function(tolua_S,"set_front_right",tolua_interfaces_MotorInterface_DriveRPMMessage_set_front_right00);
    tolua_function(tolua_S,"front_left",tolua_interfaces_MotorInterface_DriveRPMMessage_front_left00);
    tolua_function(tolua_S,"set_front_left",tolua_interfaces_MotorInterface_DriveRPMMessage_set_front_left00);
    tolua_function(tolua_S,"rear",tolua_interfaces_MotorInterface_DriveRPMMessage_rear00);
    tolua_function(tolua_S,"set_rear",tolua_interfaces_MotorInterface_DriveRPMMessage_set_rear00);
   tolua_endmodule(tolua_S);
   #ifdef __cplusplus
   tolua_cclass(tolua_S,"TransMessage","MotorInterface::TransMessage","Message",tolua_collect_MotorInterface__TransMessage);
   #else
   tolua_cclass(tolua_S,"TransMessage","MotorInterface::TransMessage","Message",NULL);
   #endif
   tolua_beginmodule(tolua_S,"TransMessage");
    tolua_function(tolua_S,"new",tolua_interfaces_MotorInterface_TransMessage_new00);
    tolua_function(tolua_S,"new_local",tolua_interfaces_MotorInterface_TransMessage_new00_local);
    tolua_function(tolua_S,".call",tolua_interfaces_MotorInterface_TransMessage_new00_local);
    tolua_function(tolua_S,"new",tolua_interfaces_MotorInterface_TransMessage_new01);
    tolua_function(tolua_S,"new_local",tolua_interfaces_MotorInterface_TransMessage_new01_local);
    tolua_function(tolua_S,".call",tolua_interfaces_MotorInterface_TransMessage_new01_local);
    tolua_function(tolua_S,"delete",tolua_interfaces_MotorInterface_TransMessage_delete00);
    tolua_function(tolua_S,"vx",tolua_interfaces_MotorInterface_TransMessage_vx00);
    tolua_function(tolua_S,"set_vx",tolua_interfaces_MotorInterface_TransMessage_set_vx00);
    tolua_function(tolua_S,"vy",tolua_interfaces_MotorInterface_TransMessage_vy00);
    tolua_function(tolua_S,"set_vy",tolua_interfaces_MotorInterface_TransMessage_set_vy00);
   tolua_endmodule(tolua_S);
   #ifdef __cplusplus
   tolua_cclass(tolua_S,"RotMessage","MotorInterface::RotMessage","Message",tolua_collect_MotorInterface__RotMessage);
   #else
   tolua_cclass(tolua_S,"RotMessage","MotorInterface::RotMessage","Message",NULL);
   #endif
   tolua_beginmodule(tolua_S,"RotMessage");
    tolua_function(tolua_S,"new",tolua_interfaces_MotorInterface_RotMessage_new00);
    tolua_function(tolua_S,"new_local",tolua_interfaces_MotorInterface_RotMessage_new00_local);
    tolua_function(tolua_S,".call",tolua_interfaces_MotorInterface_RotMessage_new00_local);
    tolua_function(tolua_S,"new",tolua_interfaces_MotorInterface_RotMessage_new01);
    tolua_function(tolua_S,"new_local",tolua_interfaces_MotorInterface_RotMessage_new01_local);
    tolua_function(tolua_S,".call",tolua_interfaces_MotorInterface_RotMessage_new01_local);
    tolua_function(tolua_S,"delete",tolua_interfaces_MotorInterface_RotMessage_delete00);
    tolua_function(tolua_S,"omega",tolua_interfaces_MotorInterface_RotMessage_omega00);
    tolua_function(tolua_S,"set_omega",tolua_interfaces_MotorInterface_RotMessage_set_omega00);
   tolua_endmodule(tolua_S);
   #ifdef __cplusplus
   tolua_cclass(tolua_S,"TransRotMessage","MotorInterface::TransRotMessage","Message",tolua_collect_MotorInterface__TransRotMessage);
   #else
   tolua_cclass(tolua_S,"TransRotMessage","MotorInterface::TransRotMessage","Message",NULL);
   #endif
   tolua_beginmodule(tolua_S,"TransRotMessage");
    tolua_function(tolua_S,"new",tolua_interfaces_MotorInterface_TransRotMessage_new00);
    tolua_function(tolua_S,"new_local",tolua_interfaces_MotorInterface_TransRotMessage_new00_local);
    tolua_function(tolua_S,".call",tolua_interfaces_MotorInterface_TransRotMessage_new00_local);
    tolua_function(tolua_S,"new",tolua_interfaces_MotorInterface_TransRotMessage_new01);
    tolua_function(tolua_S,"new_local",tolua_interfaces_MotorInterface_TransRotMessage_new01_local);
    tolua_function(tolua_S,".call",tolua_interfaces_MotorInterface_TransRotMessage_new01_local);
    tolua_function(tolua_S,"delete",tolua_interfaces_MotorInterface_TransRotMessage_delete00);
    tolua_function(tolua_S,"vx",tolua_interfaces_MotorInterface_TransRotMessage_vx00);
    tolua_function(tolua_S,"set_vx",tolua_interfaces_MotorInterface_TransRotMessage_set_vx00);
    tolua_function(tolua_S,"vy",tolua_interfaces_MotorInterface_TransRotMessage_vy00);
    tolua_function(tolua_S,"set_vy",tolua_interfaces_MotorInterface_TransRotMessage_set_vy00);
    tolua_function(tolua_S,"omega",tolua_interfaces_MotorInterface_TransRotMessage_omega00);
    tolua_function(tolua_S,"set_omega",tolua_interfaces_MotorInterface_TransRotMessage_set_omega00);
   tolua_endmodule(tolua_S);
   #ifdef __cplusplus
   tolua_cclass(tolua_S,"OrbitMessage","MotorInterface::OrbitMessage","Message",tolua_collect_MotorInterface__OrbitMessage);
   #else
   tolua_cclass(tolua_S,"OrbitMessage","MotorInterface::OrbitMessage","Message",NULL);
   #endif
   tolua_beginmodule(tolua_S,"OrbitMessage");
    tolua_function(tolua_S,"new",tolua_interfaces_MotorInterface_OrbitMessage_new00);
    tolua_function(tolua_S,"new_local",tolua_interfaces_MotorInterface_OrbitMessage_new00_local);
    tolua_function(tolua_S,".call",tolua_interfaces_MotorInterface_OrbitMessage_new00_local);
    tolua_function(tolua_S,"new",tolua_interfaces_MotorInterface_OrbitMessage_new01);
    tolua_function(tolua_S,"new_local",tolua_interfaces_MotorInterface_OrbitMessage_new01_local);
    tolua_function(tolua_S,".call",tolua_interfaces_MotorInterface_OrbitMessage_new01_local);
    tolua_function(tolua_S,"delete",tolua_interfaces_MotorInterface_OrbitMessage_delete00);
    tolua_function(tolua_S,"px",tolua_interfaces_MotorInterface_OrbitMessage_px00);
    tolua_function(tolua_S,"set_px",tolua_interfaces_MotorInterface_OrbitMessage_set_px00);
    tolua_function(tolua_S,"py",tolua_interfaces_MotorInterface_OrbitMessage_py00);
    tolua_function(tolua_S,"set_py",tolua_interfaces_MotorInterface_OrbitMessage_set_py00);
    tolua_function(tolua_S,"omega",tolua_interfaces_MotorInterface_OrbitMessage_omega00);
    tolua_function(tolua_S,"set_omega",tolua_interfaces_MotorInterface_OrbitMessage_set_omega00);
   tolua_endmodule(tolua_S);
   #ifdef __cplusplus
   tolua_cclass(tolua_S,"LinTransRotMessage","MotorInterface::LinTransRotMessage","Message",tolua_collect_MotorInterface__LinTransRotMessage);
   #else
   tolua_cclass(tolua_S,"LinTransRotMessage","MotorInterface::LinTransRotMessage","Message",NULL);
   #endif
   tolua_beginmodule(tolua_S,"LinTransRotMessage");
    tolua_function(tolua_S,"new",tolua_interfaces_MotorInterface_LinTransRotMessage_new00);
    tolua_function(tolua_S,"new_local",tolua_interfaces_MotorInterface_LinTransRotMessage_new00_local);
    tolua_function(tolua_S,".call",tolua_interfaces_MotorInterface_LinTransRotMessage_new00_local);
    tolua_function(tolua_S,"new",tolua_interfaces_MotorInterface_LinTransRotMessage_new01);
    tolua_function(tolua_S,"new_local",tolua_interfaces_MotorInterface_LinTransRotMessage_new01_local);
    tolua_function(tolua_S,".call",tolua_interfaces_MotorInterface_LinTransRotMessage_new01_local);
    tolua_function(tolua_S,"delete",tolua_interfaces_MotorInterface_LinTransRotMessage_delete00);
    tolua_function(tolua_S,"vx",tolua_interfaces_MotorInterface_LinTransRotMessage_vx00);
    tolua_function(tolua_S,"set_vx",tolua_interfaces_MotorInterface_LinTransRotMessage_set_vx00);
    tolua_function(tolua_S,"vy",tolua_interfaces_MotorInterface_LinTransRotMessage_vy00);
    tolua_function(tolua_S,"set_vy",tolua_interfaces_MotorInterface_LinTransRotMessage_set_vy00);
    tolua_function(tolua_S,"omega",tolua_interfaces_MotorInterface_LinTransRotMessage_omega00);
    tolua_function(tolua_S,"set_omega",tolua_interfaces_MotorInterface_LinTransRotMessage_set_omega00);
   tolua_endmodule(tolua_S);
   tolua_function(tolua_S,"motor_state",tolua_interfaces_MotorInterface_motor_state00);
   tolua_function(tolua_S,"set_motor_state",tolua_interfaces_MotorInterface_set_motor_state00);
   tolua_function(tolua_S,"drive_mode",tolua_interfaces_MotorInterface_drive_mode00);
   tolua_function(tolua_S,"set_drive_mode",tolua_interfaces_MotorInterface_set_drive_mode00);
   tolua_function(tolua_S,"right_rpm",tolua_interfaces_MotorInterface_right_rpm00);
   tolua_function(tolua_S,"set_right_rpm",tolua_interfaces_MotorInterface_set_right_rpm00);
   tolua_function(tolua_S,"rear_rpm",tolua_interfaces_MotorInterface_rear_rpm00);
   tolua_function(tolua_S,"set_rear_rpm",tolua_interfaces_MotorInterface_set_rear_rpm00);
   tolua_function(tolua_S,"left_rpm",tolua_interfaces_MotorInterface_left_rpm00);
   tolua_function(tolua_S,"set_left_rpm",tolua_interfaces_MotorInterface_set_left_rpm00);
   tolua_function(tolua_S,"odometry_path_length",tolua_interfaces_MotorInterface_odometry_path_length00);
   tolua_function(tolua_S,"set_odometry_path_length",tolua_interfaces_MotorInterface_set_odometry_path_length00);
   tolua_function(tolua_S,"odometry_position_x",tolua_interfaces_MotorInterface_odometry_position_x00);
   tolua_function(tolua_S,"set_odometry_position_x",tolua_interfaces_MotorInterface_set_odometry_position_x00);
   tolua_function(tolua_S,"odometry_position_y",tolua_interfaces_MotorInterface_odometry_position_y00);
   tolua_function(tolua_S,"set_odometry_position_y",tolua_interfaces_MotorInterface_set_odometry_position_y00);
   tolua_function(tolua_S,"odometry_orientation",tolua_interfaces_MotorInterface_odometry_orientation00);
   tolua_function(tolua_S,"set_odometry_orientation",tolua_interfaces_MotorInterface_set_odometry_orientation00);
   tolua_function(tolua_S,"vx",tolua_interfaces_MotorInterface_vx00);
   tolua_function(tolua_S,"set_vx",tolua_interfaces_MotorInterface_set_vx00);
   tolua_function(tolua_S,"vy",tolua_interfaces_MotorInterface_vy00);
   tolua_function(tolua_S,"set_vy",tolua_interfaces_MotorInterface_set_vy00);
   tolua_function(tolua_S,"omega",tolua_interfaces_MotorInterface_omega00);
   tolua_function(tolua_S,"set_omega",tolua_interfaces_MotorInterface_set_omega00);
   tolua_function(tolua_S,"controller_thread_id",tolua_interfaces_MotorInterface_controller_thread_id00);
   tolua_function(tolua_S,"set_controller_thread_id",tolua_interfaces_MotorInterface_set_controller_thread_id00);
   tolua_function(tolua_S,"controller_thread_name",tolua_interfaces_MotorInterface_controller_thread_name00);
   tolua_function(tolua_S,"set_controller_thread_name",tolua_interfaces_MotorInterface_set_controller_thread_name00);
  tolua_endmodule(tolua_S);
  tolua_cclass(tolua_S,"NavigatorInterface","NavigatorInterface","Interface",NULL);
  tolua_beginmodule(tolua_S,"NavigatorInterface");
   #ifdef __cplusplus
   tolua_cclass(tolua_S,"TargetMessage","NavigatorInterface::TargetMessage","Message",tolua_collect_NavigatorInterface__TargetMessage);
   #else
   tolua_cclass(tolua_S,"TargetMessage","NavigatorInterface::TargetMessage","Message",NULL);
   #endif
   tolua_beginmodule(tolua_S,"TargetMessage");
    tolua_function(tolua_S,"new",tolua_interfaces_NavigatorInterface_TargetMessage_new00);
    tolua_function(tolua_S,"new_local",tolua_interfaces_NavigatorInterface_TargetMessage_new00_local);
    tolua_function(tolua_S,".call",tolua_interfaces_NavigatorInterface_TargetMessage_new00_local);
    tolua_function(tolua_S,"new",tolua_interfaces_NavigatorInterface_TargetMessage_new01);
    tolua_function(tolua_S,"new_local",tolua_interfaces_NavigatorInterface_TargetMessage_new01_local);
    tolua_function(tolua_S,".call",tolua_interfaces_NavigatorInterface_TargetMessage_new01_local);
    tolua_function(tolua_S,"delete",tolua_interfaces_NavigatorInterface_TargetMessage_delete00);
    tolua_function(tolua_S,"x",tolua_interfaces_NavigatorInterface_TargetMessage_x00);
    tolua_function(tolua_S,"set_x",tolua_interfaces_NavigatorInterface_TargetMessage_set_x00);
    tolua_function(tolua_S,"y",tolua_interfaces_NavigatorInterface_TargetMessage_y00);
    tolua_function(tolua_S,"set_y",tolua_interfaces_NavigatorInterface_TargetMessage_set_y00);
    tolua_function(tolua_S,"orientation",tolua_interfaces_NavigatorInterface_TargetMessage_orientation00);
    tolua_function(tolua_S,"set_orientation",tolua_interfaces_NavigatorInterface_TargetMessage_set_orientation00);
   tolua_endmodule(tolua_S);
   #ifdef __cplusplus
   tolua_cclass(tolua_S,"MaxVelocityMessage","NavigatorInterface::MaxVelocityMessage","Message",tolua_collect_NavigatorInterface__MaxVelocityMessage);
   #else
   tolua_cclass(tolua_S,"MaxVelocityMessage","NavigatorInterface::MaxVelocityMessage","Message",NULL);
   #endif
   tolua_beginmodule(tolua_S,"MaxVelocityMessage");
    tolua_function(tolua_S,"new",tolua_interfaces_NavigatorInterface_MaxVelocityMessage_new00);
    tolua_function(tolua_S,"new_local",tolua_interfaces_NavigatorInterface_MaxVelocityMessage_new00_local);
    tolua_function(tolua_S,".call",tolua_interfaces_NavigatorInterface_MaxVelocityMessage_new00_local);
    tolua_function(tolua_S,"new",tolua_interfaces_NavigatorInterface_MaxVelocityMessage_new01);
    tolua_function(tolua_S,"new_local",tolua_interfaces_NavigatorInterface_MaxVelocityMessage_new01_local);
    tolua_function(tolua_S,".call",tolua_interfaces_NavigatorInterface_MaxVelocityMessage_new01_local);
    tolua_function(tolua_S,"delete",tolua_interfaces_NavigatorInterface_MaxVelocityMessage_delete00);
    tolua_function(tolua_S,"velocity",tolua_interfaces_NavigatorInterface_MaxVelocityMessage_velocity00);
    tolua_function(tolua_S,"set_velocity",tolua_interfaces_NavigatorInterface_MaxVelocityMessage_set_velocity00);
   tolua_endmodule(tolua_S);
   #ifdef __cplusplus
   tolua_cclass(tolua_S,"ObstacleMessage","NavigatorInterface::ObstacleMessage","Message",tolua_collect_NavigatorInterface__ObstacleMessage);
   #else
   tolua_cclass(tolua_S,"ObstacleMessage","NavigatorInterface::ObstacleMessage","Message",NULL);
   #endif
   tolua_beginmodule(tolua_S,"ObstacleMessage");
    tolua_function(tolua_S,"new",tolua_interfaces_NavigatorInterface_ObstacleMessage_new00);
    tolua_function(tolua_S,"new_local",tolua_interfaces_NavigatorInterface_ObstacleMessage_new00_local);
    tolua_function(tolua_S,".call",tolua_interfaces_NavigatorInterface_ObstacleMessage_new00_local);
    tolua_function(tolua_S,"new",tolua_interfaces_NavigatorInterface_ObstacleMessage_new01);
    tolua_function(tolua_S,"new_local",tolua_interfaces_NavigatorInterface_ObstacleMessage_new01_local);
    tolua_function(tolua_S,".call",tolua_interfaces_NavigatorInterface_ObstacleMessage_new01_local);
    tolua_function(tolua_S,"delete",tolua_interfaces_NavigatorInterface_ObstacleMessage_delete00);
    tolua_function(tolua_S,"x",tolua_interfaces_NavigatorInterface_ObstacleMessage_x00);
    tolua_function(tolua_S,"set_x",tolua_interfaces_NavigatorInterface_ObstacleMessage_set_x00);
    tolua_function(tolua_S,"y",tolua_interfaces_NavigatorInterface_ObstacleMessage_y00);
    tolua_function(tolua_S,"set_y",tolua_interfaces_NavigatorInterface_ObstacleMessage_set_y00);
    tolua_function(tolua_S,"width",tolua_interfaces_NavigatorInterface_ObstacleMessage_width00);
    tolua_function(tolua_S,"set_width",tolua_interfaces_NavigatorInterface_ObstacleMessage_set_width00);
   tolua_endmodule(tolua_S);
   tolua_function(tolua_S,"foo",tolua_interfaces_NavigatorInterface_foo00);
   tolua_function(tolua_S,"set_foo",tolua_interfaces_NavigatorInterface_set_foo00);
  tolua_endmodule(tolua_S);
  tolua_cclass(tolua_S,"ObjectPositionInterface","ObjectPositionInterface","Interface",NULL);
  tolua_beginmodule(tolua_S,"ObjectPositionInterface");
   tolua_variable(tolua_S,"OTHER",tolua_get_ObjectPositionInterface_unsigned_OTHER,NULL);
   tolua_variable(tolua_S,"BALL",tolua_get_ObjectPositionInterface_unsigned_BALL,NULL);
   tolua_variable(tolua_S,"OPPONENT",tolua_get_ObjectPositionInterface_unsigned_OPPONENT,NULL);
   tolua_variable(tolua_S,"TEAMMEMBER",tolua_get_ObjectPositionInterface_unsigned_TEAMMEMBER,NULL);
   tolua_function(tolua_S,"object_type",tolua_interfaces_ObjectPositionInterface_object_type00);
   tolua_function(tolua_S,"set_object_type",tolua_interfaces_ObjectPositionInterface_set_object_type00);
   tolua_function(tolua_S,"supports_relative",tolua_interfaces_ObjectPositionInterface_supports_relative00);
   tolua_function(tolua_S,"set_supports_relative",tolua_interfaces_ObjectPositionInterface_set_supports_relative00);
   tolua_function(tolua_S,"supports_global",tolua_interfaces_ObjectPositionInterface_supports_global00);
   tolua_function(tolua_S,"set_supports_global",tolua_interfaces_ObjectPositionInterface_set_supports_global00);
   tolua_function(tolua_S,"has_relative",tolua_interfaces_ObjectPositionInterface_has_relative00);
   tolua_function(tolua_S,"set_has_relative",tolua_interfaces_ObjectPositionInterface_set_has_relative00);
   tolua_function(tolua_S,"has_global",tolua_interfaces_ObjectPositionInterface_has_global00);
   tolua_function(tolua_S,"set_has_global",tolua_interfaces_ObjectPositionInterface_set_has_global00);
   tolua_function(tolua_S,"is_visible",tolua_interfaces_ObjectPositionInterface_is_visible00);
   tolua_function(tolua_S,"set_visible",tolua_interfaces_ObjectPositionInterface_set_visible00);
   tolua_function(tolua_S,"yaw",tolua_interfaces_ObjectPositionInterface_yaw00);
   tolua_function(tolua_S,"set_yaw",tolua_interfaces_ObjectPositionInterface_set_yaw00);
   tolua_function(tolua_S,"pitch",tolua_interfaces_ObjectPositionInterface_pitch00);
   tolua_function(tolua_S,"set_pitch",tolua_interfaces_ObjectPositionInterface_set_pitch00);
   tolua_function(tolua_S,"distance",tolua_interfaces_ObjectPositionInterface_distance00);
   tolua_function(tolua_S,"set_distance",tolua_interfaces_ObjectPositionInterface_set_distance00);
   tolua_function(tolua_S,"dyp_covariance",tolua_interfaces_ObjectPositionInterface_dyp_covariance00);
   tolua_function(tolua_S,"set_dyp_covariance",tolua_interfaces_ObjectPositionInterface_set_dyp_covariance00);
   tolua_function(tolua_S,"world_x",tolua_interfaces_ObjectPositionInterface_world_x00);
   tolua_function(tolua_S,"set_world_x",tolua_interfaces_ObjectPositionInterface_set_world_x00);
   tolua_function(tolua_S,"world_y",tolua_interfaces_ObjectPositionInterface_world_y00);
   tolua_function(tolua_S,"set_world_y",tolua_interfaces_ObjectPositionInterface_set_world_y00);
   tolua_function(tolua_S,"world_z",tolua_interfaces_ObjectPositionInterface_world_z00);
   tolua_function(tolua_S,"set_world_z",tolua_interfaces_ObjectPositionInterface_set_world_z00);
   tolua_function(tolua_S,"relative_x",tolua_interfaces_ObjectPositionInterface_relative_x00);
   tolua_function(tolua_S,"set_relative_x",tolua_interfaces_ObjectPositionInterface_set_relative_x00);
   tolua_function(tolua_S,"relative_y",tolua_interfaces_ObjectPositionInterface_relative_y00);
   tolua_function(tolua_S,"set_relative_y",tolua_interfaces_ObjectPositionInterface_set_relative_y00);
   tolua_function(tolua_S,"relative_z",tolua_interfaces_ObjectPositionInterface_relative_z00);
   tolua_function(tolua_S,"set_relative_z",tolua_interfaces_ObjectPositionInterface_set_relative_z00);
   tolua_function(tolua_S,"xyz_covariance",tolua_interfaces_ObjectPositionInterface_xyz_covariance00);
   tolua_function(tolua_S,"set_xyz_covariance",tolua_interfaces_ObjectPositionInterface_set_xyz_covariance00);
   tolua_function(tolua_S,"extent",tolua_interfaces_ObjectPositionInterface_extent00);
   tolua_function(tolua_S,"set_extent",tolua_interfaces_ObjectPositionInterface_set_extent00);
   tolua_function(tolua_S,"yaw_velocity",tolua_interfaces_ObjectPositionInterface_yaw_velocity00);
   tolua_function(tolua_S,"set_yaw_velocity",tolua_interfaces_ObjectPositionInterface_set_yaw_velocity00);
   tolua_function(tolua_S,"pitch_velocity",tolua_interfaces_ObjectPositionInterface_pitch_velocity00);
   tolua_function(tolua_S,"set_pitch_velocity",tolua_interfaces_ObjectPositionInterface_set_pitch_velocity00);
   tolua_function(tolua_S,"distance_velocity",tolua_interfaces_ObjectPositionInterface_distance_velocity00);
   tolua_function(tolua_S,"set_distance_velocity",tolua_interfaces_ObjectPositionInterface_set_distance_velocity00);
   tolua_function(tolua_S,"dyp_velocity_covariance",tolua_interfaces_ObjectPositionInterface_dyp_velocity_covariance00);
   tolua_function(tolua_S,"set_dyp_velocity_covariance",tolua_interfaces_ObjectPositionInterface_set_dyp_velocity_covariance00);
   tolua_function(tolua_S,"world_x_velocity",tolua_interfaces_ObjectPositionInterface_world_x_velocity00);
   tolua_function(tolua_S,"set_world_x_velocity",tolua_interfaces_ObjectPositionInterface_set_world_x_velocity00);
   tolua_function(tolua_S,"world_y_velocity",tolua_interfaces_ObjectPositionInterface_world_y_velocity00);
   tolua_function(tolua_S,"set_world_y_velocity",tolua_interfaces_ObjectPositionInterface_set_world_y_velocity00);
   tolua_function(tolua_S,"world_z_velocity",tolua_interfaces_ObjectPositionInterface_world_z_velocity00);
   tolua_function(tolua_S,"set_world_z_velocity",tolua_interfaces_ObjectPositionInterface_set_world_z_velocity00);
   tolua_function(tolua_S,"xyz_velocity_covariance",tolua_interfaces_ObjectPositionInterface_xyz_velocity_covariance00);
   tolua_function(tolua_S,"set_xyz_velocity_covariance",tolua_interfaces_ObjectPositionInterface_set_xyz_velocity_covariance00);
  tolua_endmodule(tolua_S);
  tolua_cclass(tolua_S,"SkillerInterface","SkillerInterface","Interface",NULL);
  tolua_beginmodule(tolua_S,"SkillerInterface");
   #ifdef __cplusplus
   tolua_cclass(tolua_S,"CallSkillMessage","SkillerInterface::CallSkillMessage","Message",tolua_collect_SkillerInterface__CallSkillMessage);
   #else
   tolua_cclass(tolua_S,"CallSkillMessage","SkillerInterface::CallSkillMessage","Message",NULL);
   #endif
   tolua_beginmodule(tolua_S,"CallSkillMessage");
    tolua_function(tolua_S,"new",tolua_interfaces_SkillerInterface_CallSkillMessage_new00);
    tolua_function(tolua_S,"new_local",tolua_interfaces_SkillerInterface_CallSkillMessage_new00_local);
    tolua_function(tolua_S,".call",tolua_interfaces_SkillerInterface_CallSkillMessage_new00_local);
    tolua_function(tolua_S,"new",tolua_interfaces_SkillerInterface_CallSkillMessage_new01);
    tolua_function(tolua_S,"new_local",tolua_interfaces_SkillerInterface_CallSkillMessage_new01_local);
    tolua_function(tolua_S,".call",tolua_interfaces_SkillerInterface_CallSkillMessage_new01_local);
    tolua_function(tolua_S,"delete",tolua_interfaces_SkillerInterface_CallSkillMessage_delete00);
    tolua_function(tolua_S,"skill_string",tolua_interfaces_SkillerInterface_CallSkillMessage_skill_string00);
    tolua_function(tolua_S,"set_skill_string",tolua_interfaces_SkillerInterface_CallSkillMessage_set_skill_string00);
   tolua_endmodule(tolua_S);
   #ifdef __cplusplus
   tolua_cclass(tolua_S,"RestartInterpreterMessage","SkillerInterface::RestartInterpreterMessage","Message",tolua_collect_SkillerInterface__RestartInterpreterMessage);
   #else
   tolua_cclass(tolua_S,"RestartInterpreterMessage","SkillerInterface::RestartInterpreterMessage","Message",NULL);
   #endif
   tolua_beginmodule(tolua_S,"RestartInterpreterMessage");
    tolua_function(tolua_S,"new",tolua_interfaces_SkillerInterface_RestartInterpreterMessage_new00);
    tolua_function(tolua_S,"new_local",tolua_interfaces_SkillerInterface_RestartInterpreterMessage_new00_local);
    tolua_function(tolua_S,".call",tolua_interfaces_SkillerInterface_RestartInterpreterMessage_new00_local);
    tolua_function(tolua_S,"delete",tolua_interfaces_SkillerInterface_RestartInterpreterMessage_delete00);
   tolua_endmodule(tolua_S);
   tolua_function(tolua_S,"skill_string",tolua_interfaces_SkillerInterface_skill_string00);
   tolua_function(tolua_S,"set_skill_string",tolua_interfaces_SkillerInterface_set_skill_string00);
  tolua_endmodule(tolua_S);
  tolua_cclass(tolua_S,"TestInterface","TestInterface","Interface",NULL);
  tolua_beginmodule(tolua_S,"TestInterface");
   tolua_variable(tolua_S,"TEST_CONSTANT",tolua_get_TestInterface_TEST_CONSTANT,NULL);
   tolua_variable(tolua_S,"TEST_FLOAT_CONSTANT",tolua_get_TestInterface_TEST_FLOAT_CONSTANT,NULL);
   tolua_constant(tolua_S,"TEST_ENUM_1",TestInterface::TEST_ENUM_1);
   tolua_constant(tolua_S,"TEST_ENUM_2",TestInterface::TEST_ENUM_2);
   #ifdef __cplusplus
   tolua_cclass(tolua_S,"SetTestIntMessage","TestInterface::SetTestIntMessage","Message",tolua_collect_TestInterface__SetTestIntMessage);
   #else
   tolua_cclass(tolua_S,"SetTestIntMessage","TestInterface::SetTestIntMessage","Message",NULL);
   #endif
   tolua_beginmodule(tolua_S,"SetTestIntMessage");
    tolua_function(tolua_S,"new",tolua_interfaces_TestInterface_SetTestIntMessage_new00);
    tolua_function(tolua_S,"new_local",tolua_interfaces_TestInterface_SetTestIntMessage_new00_local);
    tolua_function(tolua_S,".call",tolua_interfaces_TestInterface_SetTestIntMessage_new00_local);
    tolua_function(tolua_S,"new",tolua_interfaces_TestInterface_SetTestIntMessage_new01);
    tolua_function(tolua_S,"new_local",tolua_interfaces_TestInterface_SetTestIntMessage_new01_local);
    tolua_function(tolua_S,".call",tolua_interfaces_TestInterface_SetTestIntMessage_new01_local);
    tolua_function(tolua_S,"delete",tolua_interfaces_TestInterface_SetTestIntMessage_delete00);
    tolua_function(tolua_S,"test_int",tolua_interfaces_TestInterface_SetTestIntMessage_test_int00);
    tolua_function(tolua_S,"set_test_int",tolua_interfaces_TestInterface_SetTestIntMessage_set_test_int00);
   tolua_endmodule(tolua_S);
   #ifdef __cplusplus
   tolua_cclass(tolua_S,"SetTestStringMessage","TestInterface::SetTestStringMessage","Message",tolua_collect_TestInterface__SetTestStringMessage);
   #else
   tolua_cclass(tolua_S,"SetTestStringMessage","TestInterface::SetTestStringMessage","Message",NULL);
   #endif
   tolua_beginmodule(tolua_S,"SetTestStringMessage");
    tolua_function(tolua_S,"new",tolua_interfaces_TestInterface_SetTestStringMessage_new00);
    tolua_function(tolua_S,"new_local",tolua_interfaces_TestInterface_SetTestStringMessage_new00_local);
    tolua_function(tolua_S,".call",tolua_interfaces_TestInterface_SetTestStringMessage_new00_local);
    tolua_function(tolua_S,"new",tolua_interfaces_TestInterface_SetTestStringMessage_new01);
    tolua_function(tolua_S,"new_local",tolua_interfaces_TestInterface_SetTestStringMessage_new01_local);
    tolua_function(tolua_S,".call",tolua_interfaces_TestInterface_SetTestStringMessage_new01_local);
    tolua_function(tolua_S,"delete",tolua_interfaces_TestInterface_SetTestStringMessage_delete00);
    tolua_function(tolua_S,"test_string",tolua_interfaces_TestInterface_SetTestStringMessage_test_string00);
    tolua_function(tolua_S,"set_test_string",tolua_interfaces_TestInterface_SetTestStringMessage_set_test_string00);
   tolua_endmodule(tolua_S);
   #ifdef __cplusplus
   tolua_cclass(tolua_S,"CalculateMessage","TestInterface::CalculateMessage","Message",tolua_collect_TestInterface__CalculateMessage);
   #else
   tolua_cclass(tolua_S,"CalculateMessage","TestInterface::CalculateMessage","Message",NULL);
   #endif
   tolua_beginmodule(tolua_S,"CalculateMessage");
    tolua_function(tolua_S,"new",tolua_interfaces_TestInterface_CalculateMessage_new00);
    tolua_function(tolua_S,"new_local",tolua_interfaces_TestInterface_CalculateMessage_new00_local);
    tolua_function(tolua_S,".call",tolua_interfaces_TestInterface_CalculateMessage_new00_local);
    tolua_function(tolua_S,"new",tolua_interfaces_TestInterface_CalculateMessage_new01);
    tolua_function(tolua_S,"new_local",tolua_interfaces_TestInterface_CalculateMessage_new01_local);
    tolua_function(tolua_S,".call",tolua_interfaces_TestInterface_CalculateMessage_new01_local);
    tolua_function(tolua_S,"delete",tolua_interfaces_TestInterface_CalculateMessage_delete00);
    tolua_function(tolua_S,"summand",tolua_interfaces_TestInterface_CalculateMessage_summand00);
    tolua_function(tolua_S,"set_summand",tolua_interfaces_TestInterface_CalculateMessage_set_summand00);
    tolua_function(tolua_S,"addend",tolua_interfaces_TestInterface_CalculateMessage_addend00);
    tolua_function(tolua_S,"set_addend",tolua_interfaces_TestInterface_CalculateMessage_set_addend00);
   tolua_endmodule(tolua_S);
   tolua_function(tolua_S,"is_test_bool",tolua_interfaces_TestInterface_is_test_bool00);
   tolua_function(tolua_S,"set_test_bool",tolua_interfaces_TestInterface_set_test_bool00);
   tolua_function(tolua_S,"test_int",tolua_interfaces_TestInterface_test_int00);
   tolua_function(tolua_S,"set_test_int",tolua_interfaces_TestInterface_set_test_int00);
   tolua_function(tolua_S,"_flags",tolua_interfaces_TestInterface__flags00);
   tolua_function(tolua_S,"set__flags",tolua_interfaces_TestInterface_set__flags00);
   tolua_function(tolua_S,"test_string",tolua_interfaces_TestInterface_test_string00);
   tolua_function(tolua_S,"set_test_string",tolua_interfaces_TestInterface_set_test_string00);
   tolua_function(tolua_S,"result",tolua_interfaces_TestInterface_result00);
   tolua_function(tolua_S,"set_result",tolua_interfaces_TestInterface_set_result00);
   tolua_function(tolua_S,"test_uint",tolua_interfaces_TestInterface_test_uint00);
   tolua_function(tolua_S,"set_test_uint",tolua_interfaces_TestInterface_set_test_uint00);
   tolua_function(tolua_S,"test_ulint",tolua_interfaces_TestInterface_test_ulint00);
   tolua_function(tolua_S,"set_test_ulint",tolua_interfaces_TestInterface_set_test_ulint00);
   tolua_function(tolua_S,"test_lint",tolua_interfaces_TestInterface_test_lint00);
   tolua_function(tolua_S,"set_test_lint",tolua_interfaces_TestInterface_set_test_lint00);
  tolua_endmodule(tolua_S);
 tolua_endmodule(tolua_S);
 return 1;
}


extern "C" {
#if defined(LUA_VERSION_NUM) && LUA_VERSION_NUM >= 501
 TOLUA_API int luaopen_interfaces (lua_State* tolua_S) {
 return tolua_interfaces_open(tolua_S);
};
#endif
}


