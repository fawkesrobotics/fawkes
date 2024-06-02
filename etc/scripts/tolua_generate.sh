#!/bin/bash
# Helper script for cmake to generate c++ lua bindings
# args: 1. output path
# args: 2. target
# args: 3. toluaext path
# args: 4. GPL_WRE license
# args: 5. path to tolua files
# args: 6.-n. names of tolua files.

output_path=$1
shift
target=$1
shift
toluaext=$1
shift
license=$1
shift
tolua_file_path=$1
shift
cat "${tolua_file_path}/$1" > "${output_path}/${target}_tolua.pkg"
shift
while (( "$#" )); do
  cat "${tolua_file_path}/$1" >> "${output_path}/${target}_tolua.pkg"
  shift
done
tolua++ -L "${toluaext}" -n "${target}" "${output_path}/${target}_tolua.pkg" | \
sed -e 's/^\(.*Generated automatically .*\) on .*$/\1/' | \
awk '/^#if defined/ { f=1 }; f { t = t "\n" $0 }; !f {print}; f && /^#endif/ {print "extern \"C\" {" t "\n}\n"; f=0}' | \
awk "/^\*\/$/ { print; while ((getline line < \"${license}\") > 0) print line; print \"\n#include <core/exception.h>\" }; ! /^\*\/$/ { print }" \
> "${output_path}/${target}_tolua.cpp"
