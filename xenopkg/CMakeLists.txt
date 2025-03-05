cmake_minimum_required(VERSION 3.8)
project(xenopkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  endif()


# set(CMAKE_C_COMPILER gcc)
# set(BUILD_SHARED_LIBS OFF)

if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif() 



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(xenopkg_interfaces REQUIRED)
find_package(serial REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets )
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_subdirectory(src/meca500_ethercat_cpp_library)
add_executable(xenoethercat src/xeno_node.cpp)
add_executable(xenolqrcontroller src/xeno_lqrcontroller.cpp)
add_executable(xenopublishrt src/xeno_node_rtpub.cpp)
add_executable(timer_node_rtpub src/timer_node_rtpub.cpp)
add_executable(serialpublish src/serial_node.cpp)
add_executable(mainpendulum src/main_pendulum.cpp)
add_executable(xenokalmanlqr src/xeno_kalman_lqrcontroller.cpp)
add_executable(energypublish src/energy_node.cpp)
add_executable(penduluminterface src/pendulum_interface_node.cpp)
# add_executable(ethercat_grant src/ethercat_grant.cpp)
# target_link_libraries(ethercat_grant cap)

include_directories(include)

ament_target_dependencies(penduluminterface  
  "rclcpp" 
  "sensor_msgs"
  "std_msgs"
  "xenopkg_interfaces"
  "std_srvs"
  "Qt5"
)

ament_target_dependencies(energypublish  
  "rclcpp" 
  "sensor_msgs"
  "std_msgs"
  "xenopkg_interfaces"
  "std_srvs"
)

ament_target_dependencies(xenokalmanlqr  
  "rclcpp" 
  "sensor_msgs"
  "std_msgs"
  "xenopkg_interfaces"
  "std_srvs"
)
ament_target_dependencies(mainpendulum  
  "rclcpp" 
  "sensor_msgs"
  "std_msgs"
  "xenopkg_interfaces"
  "std_srvs"
)
ament_target_dependencies(xenolqrcontroller  
  "rclcpp" 
  "sensor_msgs"
  "std_msgs"
  "xenopkg_interfaces"
  "std_srvs"
)
ament_target_dependencies(serialpublish  
  "rclcpp" 
  "sensor_msgs"
  "std_msgs"
  "std_srvs"
  "xenopkg_interfaces"
  "serial"
)

ament_target_dependencies(xenoethercat  
  "rclcpp" 
  "sensor_msgs"
  "std_msgs"
  "xenopkg_interfaces"
)
ament_target_dependencies(xenopublishrt 
  "rclcpp" 
  "sensor_msgs"
  "realtime_tools"
  "xenopkg_interfaces"
)
ament_target_dependencies(timer_node_rtpub 
  "rclcpp" 
  "sensor_msgs"
  "realtime_tools"
)
target_link_libraries(xenoethercat alchemy)
target_link_libraries(xenoethercat meca500_driver)
target_link_libraries(xenopublishrt alchemy)
target_link_libraries(serialpublish alchemy)
target_link_libraries(xenolqrcontroller alchemy)
target_link_libraries(xenokalmanlqr alchemy)
target_link_libraries(energypublish alchemy)
target_link_libraries(xenokalmanlqr Eigen3::Eigen)
target_link_libraries(penduluminterface Qt5::Core Qt5::Widgets)
# target_link_libraries(xenoethercat 
# "-Wl,-rpath /opt/ros/humble/lib/"
# )
# target_link_libraries(xenoethercat 
# "-Wl,-rpath /opt/ros/humble/lib/x86_64-linux-gnu/"
# )
# target_link_libraries(xenoethercat 
# "-Wl,-rpath /opt/ros/humble/opt/rviz_ogre_vendor/lib/"
# )


target_link_libraries(energypublish 
    "-Wl,--no-as-needed"
    "-Wl,@/usr/xenomai/lib/cobalt.wrappers"
    "-Wl,@/usr/xenomai/lib/modechk.wrappers"
    "-lalchemy"
    "-lcopperplate"
    "/usr/xenomai/lib/xenomai/bootstrap.o"
    "-Wl,--wrap=main"
    "-Wl,--dynamic-list=/usr/xenomai/lib/dynlist.ld"
    "-L/usr/xenomai/lib"
    "-lcobalt"
    "-lmodechk"
    "-lpthread"
    "-lrt"
)
target_compile_options(energypublish PUBLIC
    "-I/usr/xenomai/include/cobalt"
    "-I/usr/xenomai/include"
    "-D_GNU_SOURCE"
    "-D_REENTRANT"
    "-fasynchronous-unwind-tables"
    "-D__COBALT__"
    "-D__COBALT_WRAP__"
    "-I/usr/xenomai/include/alchemy"
)

target_link_libraries(xenokalmanlqr 
    "-Wl,--no-as-needed"
    "-Wl,@/usr/xenomai/lib/cobalt.wrappers"
    "-Wl,@/usr/xenomai/lib/modechk.wrappers"
    "-lalchemy"
    "-lcopperplate"
    "/usr/xenomai/lib/xenomai/bootstrap.o"
    "-Wl,--wrap=main"
    "-Wl,--dynamic-list=/usr/xenomai/lib/dynlist.ld"
    "-L/usr/xenomai/lib"
    "-lcobalt"
    "-lmodechk"
    "-lpthread"
    "-lrt"
)
target_compile_options(xenokalmanlqr PUBLIC
    "-I/usr/xenomai/include/cobalt"
    "-I/usr/xenomai/include"
    "-D_GNU_SOURCE"
    "-D_REENTRANT"
    "-fasynchronous-unwind-tables"
    "-D__COBALT__"
    "-D__COBALT_WRAP__"
    "-I/usr/xenomai/include/alchemy"
)

target_link_libraries(xenolqrcontroller 
    "-Wl,--no-as-needed"
    "-Wl,@/usr/xenomai/lib/cobalt.wrappers"
    "-Wl,@/usr/xenomai/lib/modechk.wrappers"
    "-lalchemy"
    "-lcopperplate"
    "/usr/xenomai/lib/xenomai/bootstrap.o"
    "-Wl,--wrap=main"
    "-Wl,--dynamic-list=/usr/xenomai/lib/dynlist.ld"
    "-L/usr/xenomai/lib"
    "-lcobalt"
    "-lmodechk"
    "-lpthread"
    "-lrt"
)
target_compile_options(xenolqrcontroller PUBLIC
    "-I/usr/xenomai/include/cobalt"
    "-I/usr/xenomai/include"
    "-D_GNU_SOURCE"
    "-D_REENTRANT"
    "-fasynchronous-unwind-tables"
    "-D__COBALT__"
    "-D__COBALT_WRAP__"
    "-I/usr/xenomai/include/alchemy"
)
target_link_libraries(serialpublish 
    "-Wl,--no-as-needed"
    "-Wl,@/usr/xenomai/lib/cobalt.wrappers"
    "-Wl,@/usr/xenomai/lib/modechk.wrappers"
    "-lalchemy"
    "-lcopperplate"
    "/usr/xenomai/lib/xenomai/bootstrap.o"
    "-Wl,--wrap=main"
    "-Wl,--dynamic-list=/usr/xenomai/lib/dynlist.ld"
    "-L/usr/xenomai/lib"
    "-lcobalt"
    "-lmodechk"
    "-lpthread"
    "-lrt"
)
target_compile_options(serialpublish PUBLIC
    "-I/usr/xenomai/include/cobalt"
    "-I/usr/xenomai/include"
    "-D_GNU_SOURCE"
    "-D_REENTRANT"
    "-fasynchronous-unwind-tables"
    "-D__COBALT__"
    "-D__COBALT_WRAP__"
    "-I/usr/xenomai/include/alchemy"
)
target_link_libraries(xenoethercat 
    "-Wl,--no-as-needed"
    "-Wl,@/usr/xenomai/lib/cobalt.wrappers"
    "-Wl,@/usr/xenomai/lib/modechk.wrappers"
    "-lalchemy"
    "-lcopperplate"
    "/usr/xenomai/lib/xenomai/bootstrap.o"
    "-Wl,--wrap=main"
    "-Wl,--dynamic-list=/usr/xenomai/lib/dynlist.ld"
    "-L/usr/xenomai/lib"
    "-lcobalt"
    "-lmodechk"
    "-lpthread"
    "-lrt"
)
target_compile_options(xenoethercat PUBLIC
    "-I/usr/xenomai/include/cobalt"
    "-I/usr/xenomai/include"
    "-D_GNU_SOURCE"
    "-D_REENTRANT"
    "-fasynchronous-unwind-tables"
    "-D__COBALT__"
    "-D__COBALT_WRAP__"
    "-I/usr/xenomai/include/alchemy"
)

target_link_libraries(xenopublishrt
    "-Wl,--no-as-needed"
    "-Wl,@/usr/xenomai/lib/cobalt.wrappers"
    "-Wl,@/usr/xenomai/lib/modechk.wrappers"
    "-lalchemy"
    "-lcopperplate"
    "/usr/xenomai/lib/xenomai/bootstrap.o"
    "-Wl,--wrap=main"
    "-Wl,--dynamic-list=/usr/xenomai/lib/dynlist.ld"
    "-L/usr/xenomai/lib"
    "-lcobalt"
    "-lmodechk"
    "-lpthread"
    "-lrt"
)
target_compile_options(xenopublishrt PUBLIC
    "-I/usr/xenomai/include/cobalt"
    "-I/usr/xenomai/include"
    "-D_GNU_SOURCE"
    "-D_REENTRANT"
    "-fasynchronous-unwind-tables"
    "-D__COBALT__"
    "-D__COBALT_WRAP__"
    "-I/usr/xenomai/include/alchemy"
)


install(TARGETS xenoethercat DESTINATION lib/${PROJECT_NAME})
install(TARGETS xenopublishrt DESTINATION lib/${PROJECT_NAME})
install(TARGETS timer_node_rtpub DESTINATION lib/${PROJECT_NAME})
install(TARGETS serialpublish DESTINATION lib/${PROJECT_NAME})
install(TARGETS xenolqrcontroller DESTINATION lib/${PROJECT_NAME})
install(TARGETS mainpendulum DESTINATION lib/${PROJECT_NAME})
install(TARGETS xenokalmanlqr DESTINATION lib/${PROJECT_NAME})
install(TARGETS energypublish DESTINATION lib/${PROJECT_NAME})
install(TARGETS penduluminterface DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})
# install(TARGETS ethercat_grant DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
