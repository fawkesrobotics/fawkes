import subprocess
import os
import sys, getopt

def main(argv):
    try:
        opts, args = getopt.getopt(argv,"hp:vebcl",["package=", "version", "exist", "build", "cflags", "lflags"])
    except getopt.GetoptError:
        sys.exit(2)
    package_name = ""
    for opt, arg in opts:
        if opt == '-h':
            print('argument description')
            sys.exit()
        elif opt in ("-p", "--package"):
            package_name = arg
        elif opt in ("-v", "--version"):
            print(1)

            sys.exit()
        elif opt in ("-e", "--exist"):
            if os.path.exists(f"/home/gjorgji/ros2_galactic/build/{package_name}"):
                print(1)
            else:
                print(0)
            sys.exit()
        elif opt in ("-b", "--build"):
            with open("/home/gjorgji/ros2_galactic/src/dummy_for_flags/CMakeLists.txt", 'w') as cmakelists_txt:
                cmakelists_txt.write(f'''\
cmake_minimum_required(VERSION 3.8)
project(dummy_for_flags)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#add_compile_options()
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package({package_name} REQUIRED)

add_executable(${{PROJECT_NAME}} src/dummy_for_flags.cpp)

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


            with open("/home/gjorgji/ros2_galactic/src/dummy_for_flags/package.xml", 'w') as package_xml:
                package_xml.write(f'''\
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
<name>dummy_for_flags</name>
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
            subprocess.call("cd /home/gjorgji/ros2_galactic/; rm -rf build/dummy_for_flags/; rm -rf install/dummy_for_flags", shell=True)
            subprocess.call("cd /home/gjorgji/ros2_galactic/; source /home/gjorgji/ros2_galactic/install/setup.bash > /dev/null 2>&1; colcon build --packages-select dummy_for_flags > /dev/null 2>&1", shell=True)
        elif opt in ("-c", "--cflags"):
            subprocess.call("cd /home/gjorgji/ros2_galactic/build/dummy_for_flags/; make clean -s", shell=True)
            build_object_command = subprocess.check_output("cd /home/gjorgji/ros2_galactic/build/dummy_for_flags/; make  -n VERBOSE=1 | grep ccache", shell=True)
            build_object_flags= [ "-" + flag.replace(":", " ").replace(",", " ") for flag in build_object_command.decode("utf-8").strip('\n').split(" -") if "dummy_for_flags" not in flag and "ccache" not in flag]
            for cflag in build_object_flags:
                print(cflag + " ", end="")
            print("")
            sys.exit()
        elif opt in ("-l", "--lflags"):
            link_command = subprocess.check_output("cd /home/gjorgji/ros2_galactic/build/dummy_for_flags/; cat CMakeFiles/dummy_for_flags.dir/link.txt", shell=True)
            link_flags= [ "-" + flag.replace(":", " ").replace(",", " ") for flag in link_command.decode("utf-8").strip('\n').split(" -") if  "dummy_for_flags" not in flag and "ccache" not in flag and "-Wl" not in flag and "-rpath" not in flag] # 
            for lflag in link_flags:
                print(lflag + " ", end="")
            print("")
            sys.exit()
    

#print("Building and linking for test...")
#subprocess.call(b"cd /home/gjorgji/ros2_galactic/build/dummy_for_flags/; "+build_object_command+b" > /dev/null ", shell=True)
#subprocess.call(b"cd /home/gjorgji/ros2_galactic/build/dummy_for_flags/; "+link_command+b" > /dev/null", shell=True)

#print("Print calling for test:")
#os.system("/home/gjorgji/ros2_galactic/build/dummy_for_flags/dummy_for_flags")

if __name__=="__main__":
    main(sys.argv[1:])
