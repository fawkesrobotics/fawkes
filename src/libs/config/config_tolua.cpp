/*
** Lua binding: config
** Generated automatically by tolua++-1.0.92
*/
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

#ifndef __cplusplus
#include "stdlib.h"
#endif
#include "string.h"

#include "tolua++.h"

/* Exported function */
TOLUA_API int  tolua_config_open (lua_State* tolua_S);

#include <config/config.h>

/* function to release collected object via destructor */
#ifdef __cplusplus

static int tolua_collect_std__list_std__string_ (lua_State* tolua_S)
{
 std::list<std::string>* self = (std::list<std::string>*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_Configuration__ValueIterator (lua_State* tolua_S)
{
 Configuration::ValueIterator* self = (Configuration::ValueIterator*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}
#endif


/* function to register type */
static void tolua_reg_types (lua_State* tolua_S)
{
 tolua_usertype(tolua_S,"Configuration::ValueIterator");
 tolua_usertype(tolua_S,"Configuration");
 tolua_usertype(tolua_S,"std::list<std::string>");
}

/* method: delete of class  ValueIterator */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_ValueIterator_delete00
static int tolua_config_Configuration_ValueIterator_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration::ValueIterator",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration::ValueIterator* self = (Configuration::ValueIterator*)  tolua_tousertype(tolua_S,1,0);
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

/* method: next of class  ValueIterator */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_ValueIterator_next00
static int tolua_config_Configuration_ValueIterator_next00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration::ValueIterator",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration::ValueIterator* self = (Configuration::ValueIterator*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'next'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->next();
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'next'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: valid of class  ValueIterator */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_ValueIterator_valid00
static int tolua_config_Configuration_ValueIterator_valid00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration::ValueIterator",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration::ValueIterator* self = (Configuration::ValueIterator*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'valid'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->valid();
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'valid'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: path of class  ValueIterator */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_ValueIterator_path00
static int tolua_config_Configuration_ValueIterator_path00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration::ValueIterator",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration::ValueIterator* self = (Configuration::ValueIterator*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'path'",NULL);
#endif
  {
   const char* tolua_ret = (const char*)  self->path();
   tolua_pushstring(tolua_S,(const char*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'path'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: type of class  ValueIterator */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_ValueIterator_type00
static int tolua_config_Configuration_ValueIterator_type00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration::ValueIterator",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration::ValueIterator* self = (Configuration::ValueIterator*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'type'",NULL);
#endif
  {
   const char* tolua_ret = (const char*)  self->type();
   tolua_pushstring(tolua_S,(const char*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'type'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: is_float of class  ValueIterator */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_ValueIterator_is_float00
static int tolua_config_Configuration_ValueIterator_is_float00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration::ValueIterator",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration::ValueIterator* self = (Configuration::ValueIterator*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'is_float'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->is_float();
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'is_float'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: is_uint of class  ValueIterator */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_ValueIterator_is_uint00
static int tolua_config_Configuration_ValueIterator_is_uint00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration::ValueIterator",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration::ValueIterator* self = (Configuration::ValueIterator*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'is_uint'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->is_uint();
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'is_uint'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: is_int of class  ValueIterator */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_ValueIterator_is_int00
static int tolua_config_Configuration_ValueIterator_is_int00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration::ValueIterator",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration::ValueIterator* self = (Configuration::ValueIterator*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'is_int'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->is_int();
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'is_int'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: is_bool of class  ValueIterator */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_ValueIterator_is_bool00
static int tolua_config_Configuration_ValueIterator_is_bool00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration::ValueIterator",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration::ValueIterator* self = (Configuration::ValueIterator*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'is_bool'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->is_bool();
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'is_bool'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: is_string of class  ValueIterator */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_ValueIterator_is_string00
static int tolua_config_Configuration_ValueIterator_is_string00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration::ValueIterator",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration::ValueIterator* self = (Configuration::ValueIterator*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'is_string'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->is_string();
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'is_string'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: get_float of class  ValueIterator */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_ValueIterator_get_float00
static int tolua_config_Configuration_ValueIterator_get_float00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration::ValueIterator",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration::ValueIterator* self = (Configuration::ValueIterator*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'get_float'",NULL);
#endif
  {
   float tolua_ret = (float)  self->get_float();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'get_float'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: get_uint of class  ValueIterator */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_ValueIterator_get_uint00
static int tolua_config_Configuration_ValueIterator_get_uint00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration::ValueIterator",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration::ValueIterator* self = (Configuration::ValueIterator*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'get_uint'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->get_uint();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'get_uint'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: get_int of class  ValueIterator */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_ValueIterator_get_int00
static int tolua_config_Configuration_ValueIterator_get_int00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration::ValueIterator",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration::ValueIterator* self = (Configuration::ValueIterator*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'get_int'",NULL);
#endif
  {
   int tolua_ret = (int)  self->get_int();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'get_int'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: get_bool of class  ValueIterator */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_ValueIterator_get_bool00
static int tolua_config_Configuration_ValueIterator_get_bool00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration::ValueIterator",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration::ValueIterator* self = (Configuration::ValueIterator*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'get_bool'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->get_bool();
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'get_bool'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: get_string of class  ValueIterator */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_ValueIterator_get_string00
static int tolua_config_Configuration_ValueIterator_get_string00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration::ValueIterator",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration::ValueIterator* self = (Configuration::ValueIterator*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'get_string'",NULL);
#endif
  {
   std::string tolua_ret = (std::string)  self->get_string();
   tolua_pushcppstring(tolua_S,(const char*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'get_string'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: is_default of class  ValueIterator */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_ValueIterator_is_default00
static int tolua_config_Configuration_ValueIterator_is_default00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration::ValueIterator",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration::ValueIterator* self = (Configuration::ValueIterator*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'is_default'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->is_default();
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'is_default'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: tag of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_tag00
static int tolua_config_Configuration_tag00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* tag = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'tag'",NULL);
#endif
  {
   self->tag(tag);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'tag'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: tags of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_tags00
static int tolua_config_Configuration_tags00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'tags'",NULL);
#endif
  {
   std::list<std::string> tolua_ret = (std::list<std::string>)  self->tags();
   {
#ifdef __cplusplus
    void* tolua_obj = new std::list<std::string>(tolua_ret);
    tolua_pushusertype_and_takeownership(tolua_S,tolua_obj,"std::list<std::string>");
#else
    void* tolua_obj = tolua_copy(tolua_S,(void*)&tolua_ret,sizeof(std::list<std::string>));
    tolua_pushusertype_and_takeownership(tolua_S,tolua_obj,"std::list<std::string>");
#endif
   }
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'tags'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: exists of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_exists00
static int tolua_config_Configuration_exists00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'exists'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->exists(path);
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'exists'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: is_float of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_is_float00
static int tolua_config_Configuration_is_float00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'is_float'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->is_float(path);
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'is_float'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: is_uint of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_is_uint00
static int tolua_config_Configuration_is_uint00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'is_uint'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->is_uint(path);
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'is_uint'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: is_int of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_is_int00
static int tolua_config_Configuration_is_int00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'is_int'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->is_int(path);
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'is_int'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: is_bool of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_is_bool00
static int tolua_config_Configuration_is_bool00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'is_bool'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->is_bool(path);
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'is_bool'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: is_string of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_is_string00
static int tolua_config_Configuration_is_string00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'is_string'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->is_string(path);
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'is_string'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: is_default of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_is_default00
static int tolua_config_Configuration_is_default00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'is_default'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->is_default(path);
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'is_default'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: get_float of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_get_float00
static int tolua_config_Configuration_get_float00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'get_float'",NULL);
#endif
  {
   float tolua_ret = (float)  self->get_float(path);
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'get_float'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: get_uint of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_get_uint00
static int tolua_config_Configuration_get_uint00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'get_uint'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->get_uint(path);
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'get_uint'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: get_int of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_get_int00
static int tolua_config_Configuration_get_int00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'get_int'",NULL);
#endif
  {
   int tolua_ret = (int)  self->get_int(path);
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'get_int'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: get_bool of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_get_bool00
static int tolua_config_Configuration_get_bool00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'get_bool'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->get_bool(path);
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'get_bool'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: get_string of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_get_string00
static int tolua_config_Configuration_get_string00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'get_string'",NULL);
#endif
  {
   std::string tolua_ret = (std::string)  self->get_string(path);
   tolua_pushcppstring(tolua_S,(const char*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'get_string'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: get_value of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_get_value00
static int tolua_config_Configuration_get_value00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'get_value'",NULL);
#endif
  {
   Configuration::ValueIterator* tolua_ret = (Configuration::ValueIterator*)  self->get_value(path);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"Configuration::ValueIterator");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'get_value'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_float of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_set_float00
static int tolua_config_Configuration_set_float00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
  float f = ((float)  tolua_tonumber(tolua_S,3,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_float'",NULL);
#endif
  {
   self->set_float(path,f);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_float'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_uint of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_set_uint00
static int tolua_config_Configuration_set_uint00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
  unsigned int uint = ((unsigned int)  tolua_tonumber(tolua_S,3,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_uint'",NULL);
#endif
  {
   self->set_uint(path,uint);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_uint'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_int of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_set_int00
static int tolua_config_Configuration_set_int00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
  int i = ((int)  tolua_tonumber(tolua_S,3,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_int'",NULL);
#endif
  {
   self->set_int(path,i);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_int'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_bool of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_set_bool00
static int tolua_config_Configuration_set_bool00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isboolean(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
  bool b = ((bool)  tolua_toboolean(tolua_S,3,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_bool'",NULL);
#endif
  {
   self->set_bool(path,b);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_bool'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_string of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_set_string00
static int tolua_config_Configuration_set_string00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_iscppstring(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
  std::string s = ((std::string)  tolua_tocppstring(tolua_S,3,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_string'",NULL);
#endif
  {
   self->set_string(path,s);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_string'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_string of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_set_string01
static int tolua_config_Configuration_set_string01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isstring(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
  const char* s = ((const char*)  tolua_tostring(tolua_S,3,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_string'",NULL);
#endif
  {
   self->set_string(path,s);
  }
 }
 return 0;
tolua_lerror:
 return tolua_config_Configuration_set_string00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: erase of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_erase00
static int tolua_config_Configuration_erase00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'erase'",NULL);
#endif
  {
   self->erase(path);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'erase'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_default_float of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_set_default_float00
static int tolua_config_Configuration_set_default_float00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
  float f = ((float)  tolua_tonumber(tolua_S,3,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_default_float'",NULL);
#endif
  {
   self->set_default_float(path,f);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_default_float'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_default_uint of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_set_default_uint00
static int tolua_config_Configuration_set_default_uint00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
  unsigned int uint = ((unsigned int)  tolua_tonumber(tolua_S,3,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_default_uint'",NULL);
#endif
  {
   self->set_default_uint(path,uint);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_default_uint'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_default_int of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_set_default_int00
static int tolua_config_Configuration_set_default_int00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
  int i = ((int)  tolua_tonumber(tolua_S,3,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_default_int'",NULL);
#endif
  {
   self->set_default_int(path,i);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_default_int'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_default_bool of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_set_default_bool00
static int tolua_config_Configuration_set_default_bool00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isboolean(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
  bool b = ((bool)  tolua_toboolean(tolua_S,3,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_default_bool'",NULL);
#endif
  {
   self->set_default_bool(path,b);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_default_bool'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_default_string of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_set_default_string00
static int tolua_config_Configuration_set_default_string00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_iscppstring(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
  std::string s = ((std::string)  tolua_tocppstring(tolua_S,3,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_default_string'",NULL);
#endif
  {
   self->set_default_string(path,s);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_default_string'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_default_string of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_set_default_string01
static int tolua_config_Configuration_set_default_string01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isstring(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
  const char* s = ((const char*)  tolua_tostring(tolua_S,3,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_default_string'",NULL);
#endif
  {
   self->set_default_string(path,s);
  }
 }
 return 0;
tolua_lerror:
 return tolua_config_Configuration_set_default_string00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: erase_default of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_erase_default00
static int tolua_config_Configuration_erase_default00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'erase_default'",NULL);
#endif
  {
   self->erase_default(path);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'erase_default'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: iterator of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_iterator00
static int tolua_config_Configuration_iterator00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'iterator'",NULL);
#endif
  {
   Configuration::ValueIterator* tolua_ret = (Configuration::ValueIterator*)  self->iterator();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"Configuration::ValueIterator");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'iterator'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: search of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_search00
static int tolua_config_Configuration_search00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
  const char* path = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'search'",NULL);
#endif
  {
   Configuration::ValueIterator* tolua_ret = (Configuration::ValueIterator*)  self->search(path);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"Configuration::ValueIterator");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'search'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: lock of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_lock00
static int tolua_config_Configuration_lock00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'lock'",NULL);
#endif
  {
   self->lock();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'lock'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: try_lock of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_try_lock00
static int tolua_config_Configuration_try_lock00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'try_lock'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->try_lock();
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'try_lock'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: unlock of class  Configuration */
#ifndef TOLUA_DISABLE_tolua_config_Configuration_unlock00
static int tolua_config_Configuration_unlock00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Configuration",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Configuration* self = (Configuration*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'unlock'",NULL);
#endif
  {
   self->unlock();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'unlock'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* Open function */
TOLUA_API int tolua_config_open (lua_State* tolua_S)
{
 tolua_open(tolua_S);
 tolua_reg_types(tolua_S);
 tolua_module(tolua_S,NULL,0);
 tolua_beginmodule(tolua_S,NULL);
  tolua_cclass(tolua_S,"Configuration","Configuration","",NULL);
  tolua_beginmodule(tolua_S,"Configuration");
   #ifdef __cplusplus
   tolua_cclass(tolua_S,"ValueIterator","Configuration::ValueIterator","",tolua_collect_Configuration__ValueIterator);
   #else
   tolua_cclass(tolua_S,"ValueIterator","Configuration::ValueIterator","",NULL);
   #endif
   tolua_beginmodule(tolua_S,"ValueIterator");
    tolua_function(tolua_S,"delete",tolua_config_Configuration_ValueIterator_delete00);
    tolua_function(tolua_S,"next",tolua_config_Configuration_ValueIterator_next00);
    tolua_function(tolua_S,"valid",tolua_config_Configuration_ValueIterator_valid00);
    tolua_function(tolua_S,"path",tolua_config_Configuration_ValueIterator_path00);
    tolua_function(tolua_S,"type",tolua_config_Configuration_ValueIterator_type00);
    tolua_function(tolua_S,"is_float",tolua_config_Configuration_ValueIterator_is_float00);
    tolua_function(tolua_S,"is_uint",tolua_config_Configuration_ValueIterator_is_uint00);
    tolua_function(tolua_S,"is_int",tolua_config_Configuration_ValueIterator_is_int00);
    tolua_function(tolua_S,"is_bool",tolua_config_Configuration_ValueIterator_is_bool00);
    tolua_function(tolua_S,"is_string",tolua_config_Configuration_ValueIterator_is_string00);
    tolua_function(tolua_S,"get_float",tolua_config_Configuration_ValueIterator_get_float00);
    tolua_function(tolua_S,"get_uint",tolua_config_Configuration_ValueIterator_get_uint00);
    tolua_function(tolua_S,"get_int",tolua_config_Configuration_ValueIterator_get_int00);
    tolua_function(tolua_S,"get_bool",tolua_config_Configuration_ValueIterator_get_bool00);
    tolua_function(tolua_S,"get_string",tolua_config_Configuration_ValueIterator_get_string00);
    tolua_function(tolua_S,"is_default",tolua_config_Configuration_ValueIterator_is_default00);
   tolua_endmodule(tolua_S);
   tolua_function(tolua_S,"tag",tolua_config_Configuration_tag00);
   tolua_function(tolua_S,"tags",tolua_config_Configuration_tags00);
   tolua_function(tolua_S,"exists",tolua_config_Configuration_exists00);
   tolua_function(tolua_S,"is_float",tolua_config_Configuration_is_float00);
   tolua_function(tolua_S,"is_uint",tolua_config_Configuration_is_uint00);
   tolua_function(tolua_S,"is_int",tolua_config_Configuration_is_int00);
   tolua_function(tolua_S,"is_bool",tolua_config_Configuration_is_bool00);
   tolua_function(tolua_S,"is_string",tolua_config_Configuration_is_string00);
   tolua_function(tolua_S,"is_default",tolua_config_Configuration_is_default00);
   tolua_function(tolua_S,"get_float",tolua_config_Configuration_get_float00);
   tolua_function(tolua_S,"get_uint",tolua_config_Configuration_get_uint00);
   tolua_function(tolua_S,"get_int",tolua_config_Configuration_get_int00);
   tolua_function(tolua_S,"get_bool",tolua_config_Configuration_get_bool00);
   tolua_function(tolua_S,"get_string",tolua_config_Configuration_get_string00);
   tolua_function(tolua_S,"get_value",tolua_config_Configuration_get_value00);
   tolua_function(tolua_S,"set_float",tolua_config_Configuration_set_float00);
   tolua_function(tolua_S,"set_uint",tolua_config_Configuration_set_uint00);
   tolua_function(tolua_S,"set_int",tolua_config_Configuration_set_int00);
   tolua_function(tolua_S,"set_bool",tolua_config_Configuration_set_bool00);
   tolua_function(tolua_S,"set_string",tolua_config_Configuration_set_string00);
   tolua_function(tolua_S,"set_string",tolua_config_Configuration_set_string01);
   tolua_function(tolua_S,"erase",tolua_config_Configuration_erase00);
   tolua_function(tolua_S,"set_default_float",tolua_config_Configuration_set_default_float00);
   tolua_function(tolua_S,"set_default_uint",tolua_config_Configuration_set_default_uint00);
   tolua_function(tolua_S,"set_default_int",tolua_config_Configuration_set_default_int00);
   tolua_function(tolua_S,"set_default_bool",tolua_config_Configuration_set_default_bool00);
   tolua_function(tolua_S,"set_default_string",tolua_config_Configuration_set_default_string00);
   tolua_function(tolua_S,"set_default_string",tolua_config_Configuration_set_default_string01);
   tolua_function(tolua_S,"erase_default",tolua_config_Configuration_erase_default00);
   tolua_function(tolua_S,"iterator",tolua_config_Configuration_iterator00);
   tolua_function(tolua_S,"search",tolua_config_Configuration_search00);
   tolua_function(tolua_S,"lock",tolua_config_Configuration_lock00);
   tolua_function(tolua_S,"try_lock",tolua_config_Configuration_try_lock00);
   tolua_function(tolua_S,"unlock",tolua_config_Configuration_unlock00);
  tolua_endmodule(tolua_S);
 tolua_endmodule(tolua_S);
 return 1;
}


extern "C" {
#if defined(LUA_VERSION_NUM) && LUA_VERSION_NUM >= 501
 TOLUA_API int luaopen_config (lua_State* tolua_S) {
 return tolua_config_open(tolua_S);
};
#endif
}


