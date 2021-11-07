import subprocess
import os
import sys, getopt
from pathlib import Path

def main(argv):
    try:
        opts, args = getopt.getopt(argv,"hw:p:vebcl",["ros2-workspace=", "package=", "version", "exist", "build", "cflags", "lflags"])
    except getopt.GetoptError:
        sys.exit(2)
    package_name = ""
    ros2_workspace = ""
    tmp_ws_dir = "/tmp/tmp_ws_for_flags"
    for opt, arg in opts:
        if opt == '-h':
            print('argument description')
            sys.exit()
        elif opt in ('-w', "--ros2-workspace"):
            ros2_workspace = arg
        elif opt in ("-p", "--package"):
            package_name = arg
        elif opt in ("-v", "--version"):
            print(1)
            sys.exit()
        elif opt in ("-e", "--exist"):
            if os.path.exists(f"{ros2_workspace}/build/{package_name}") or os.path.exists(f"/tmp/tmp_ws_for_flags/build/{package_name}"):
                print(1)
            else:
                print(0)
            sys.exit()
        elif opt in ("-b", "--build"):
            
            if os.path.exists(f"{tmp_ws_dir}/build/dummy_for_{package_name}_flags"):
                continue
            dummy_package_path = f"{tmp_ws_dir}/src/dummy_for_{package_name}_flags"
            Path(f"{dummy_package_path}/src").mkdir(parents=True, exist_ok=True)
            
            with open(f"{dummy_package_path}/CMakeLists.txt", 'w') as cmakelists_txt:
                cmakelists_txt.write(f'''\
cmake_minimum_required(VERSION 3.8)
project(dummy_for_{package_name}_flags)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#add_compile_options()
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package({package_name} REQUIRED)

add_executable(${{PROJECT_NAME}} src/dummy_for_{package_name}_flags.cpp)

ament_target_dependencies(${{PROJECT_NAME}} {package_name})

include_directories(${{PROJECT_NAME}} include ${{{package_name}_INCLUDE_DIRS}})

target_link_libraries(${{PROJECT_NAME}} ${{{package_name}_LINK_LIBRARIES}})

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
# the following line skips the linter which checks for copyrights
# uncomment the line when a copyright and license is not present in all source files
#set(ament_cmake_copyright_FOUND TRUE)
# the following line skips cpplint (only works in a git repo)
# uncomment the line when this package is not in a git repo
#set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()


# Install launch files

# Install nodes
install(
TARGETS ${{PROJECT_NAME}}
DESTINATION lib/${{PROJECT_NAME}}
)


ament_package()''')


            with open(f"{dummy_package_path}/package.xml", 'w') as package_xml:
                package_xml.write(f'''\
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
<name>dummy_for_{package_name}_flags</name>
<version>0.0.0</version>
<description>TODO: Package description</description>
<maintainer email="max.mustermann@alumni.fh-aachen.de">max</maintainer>
<license>TODO: License declaration</license>

<buildtool_depend>ament_cmake</buildtool_depend>

<depend>{package_name}</depend>

<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_lint_common</test_depend>

<export>
    <build_type>ament_cmake</build_type>
</export>
</package>''')

            with open(f"{dummy_package_path}/src/dummy_for_{package_name}_flags.cpp", 'w') as package_xml:
                package_xml.write(f'''\
int main(int argc, char *argv[])
{{
    return 0;
}}''')

            subprocess.call(f"cd {tmp_ws_dir}; rm -rf build/dummy_for_{package_name}_flags/; rm -rf install/dummy_for_{package_name}_flags", shell=True)
            subprocess.call(f"cd {tmp_ws_dir}; source {ros2_workspace}/install/setup.bash > /dev/null 2>&1; colcon build --packages-select dummy_for_{package_name}_flags > /dev/null 2>&1", shell=True)
        elif opt in ("-c", "--cflags"):
            '''subprocess.call(f"cd {tmp_ws_dir}/build/dummy_for_{package_name}_flags/; make clean -s", shell=True)
            build_object_command = subprocess.check_output(f"cd {tmp_ws_dir}/build/dummy_for_{package_name}_flags/; make  -n VERBOSE=1 | grep ccache", shell=True)
            build_object_flags= [ "-" + flag.replace(":", " ").replace(",", " ") for flag in build_object_command.decode("utf-8").strip('\n').split(" -") if f"dummy_for_{package_name}_flags" not in flag and "ccache" not in flag]
            print("old:")
            for cflag in build_object_flags:
                print(cflag + " ", end="")
            print("\nnew:")'''
            with open(f"{tmp_ws_dir}/build/dummy_for_{package_name}_flags/CMakeFiles/dummy_for_{package_name}_flags.dir/flags.make", 'r') as cf:
                compile_flags = ""
                for line in cf.readlines():
                    if any(var in line for var in ["CXX_DEFINES", "CXX_INCLUDES", "CXX_FLAGS"]):
                        compile_flags += ' '.join([flag for flag in line.split("=")[-1].replace('\n', '').split(' ') if f"dummy_for_{package_name}_flags" not in flag])
                print(compile_flags)
            sys.exit(0)
        elif opt in ("-l", "--lflags"):
            with open(f"{tmp_ws_dir}/build/dummy_for_{package_name}_flags/CMakeFiles/dummy_for_{package_name}_flags.dir/link.txt", 'r') as lf:
                link_flags = ""
                for line in lf.readlines():
                    link_flags += ' '.join([flag for flag in line.split(',')[-1].replace('\n', '').split(' ') if f"dummy_for_{package_name}_flags" not in flag])
            print(link_flags)
            sys.exit(0)

if __name__=="__main__":
    main(sys.argv[1:])
