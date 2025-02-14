# CMakeList.txt : CMake project for tflow-process-tracker, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.0)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("tflow-process-tracker")

find_package(PkgConfig REQUIRED)

# Add source to this project's executable.
add_executable (${PROJECT_NAME}
    "main.cpp"
    "tflow-buf-srv.cpp"
    "tflow-buf-srv.hpp"
    "tflow-buf-cli.cpp"
    "tflow-buf-cli.hpp"
    "tflow-buf-pck.hpp"
    "tflow-buf.hpp"
    "tflow-build-cfg.hpp"
    "tflow-common.hpp"
    "tflow-glib.hpp"
    "tflow-ctrl-cli-port.cpp"
    "tflow-ctrl-cli-port.hpp"
    "tflow-ctrl-process.cpp"
    "tflow-ctrl-process.hpp"
    "tflow-ctrl-srv.cpp"
    "tflow-ctrl-srv.hpp"
    "tflow-ctrl.cpp"
    "tflow-ctrl.hpp"
    "tflow-ctrl-srv-process.hpp"
    "tracker/tflow-trck-dashboard.cpp"
    "tracker/tflow-trck-dashboard.hpp"
    "tracker/tflow-trck-feature.cpp"
    "tracker/tflow-trck-feature.hpp"
    "tracker/tflow-trck-gftt.cpp"
    "tracker/tflow-trck-gftt.hpp"
    "tracker/tflow-trck-imu.hpp"
    "tracker/tflow-trck-cfg.cpp"
    "tracker/tflow-trck-cfg.hpp"
    "tracker/tflow-trck.cpp"
    "tracker/tflow-trck.hpp"
    "tflow-perfmon.cpp"
    "tflow-perfmon.hpp"
    "tflow-player.cpp"
    "tflow-player.hpp"
    "tflow-process.cpp"
    "tflow-process.hpp"
    "tflow-render.cpp"
    "tflow-render.hpp"
    "tflow-streamer.cpp"
    "tflow-streamer.hpp"
    "tflow-tracelog.cpp"
    "tflow-tracelog.hpp"
    "tflow-algo.hpp"
)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)
endif()

pkg_check_modules(OPENCV REQUIRED IMPORTED_TARGET
    opencv4
)

pkg_check_modules(OTHERS REQUIRED IMPORTED_TARGET
    json11
    libjpeg
)

pkg_check_modules(GLIB REQUIRED IMPORTED_TARGET 
    glib-2.0
    gmodule-2.0
    gio-2.0
    gio-unix-2.0
)

pkg_check_modules(GLIBMM REQUIRED IMPORTED_TARGET 
    glibmm-2.4
    giomm-2.4
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0") # Add debug information

#AV: On the Mickledore, the default compiling script change the build environment path 
#    path to /usr/scr/... As a result MSVC can't set a breakpoint on a source's line, 
#    while data breakpoints and stepping are working correctly.
#    Can be solved by setting MSVC "Additional Debugger Command" to restore the 
#    original path.
#    set substitute-path /usr/src/debug/tflow-process-xxx/1.0-r0 /home/av/compulab-nxp-bsp/build-ucm-imx8m-plus/workspace/sources/tflow-process-xxx

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmacro-prefix-map=/home/av/compulab-nxp-bsp/build-ucm-imx8m-plus/workspace/sources/tflow-process=/home/av/compulab-nxp-bsp/build-ucm-imx8m-plus/workspace/sources/tflow-process") # Add debug information
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdebug-prefix-map=/home/av/compulab-nxp-bsp/build-ucm-imx8m-plus/workspace/sources/tflow-process=/home/av/compulab-nxp-bsp/build-ucm-imx8m-plus/workspace/sources/tflow-process") # Add debug information

target_include_directories(${PROJECT_NAME} PRIVATE ${OPENCV_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PRIVATE ${OTHERS_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PRIVATE ${GLIB_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PRIVATE ${GLIBMM_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PkgConfig::OPENCV)
target_link_libraries(${PROJECT_NAME} PkgConfig::OTHERS)
target_link_libraries(${PROJECT_NAME} PkgConfig::GLIB)
target_link_libraries(${PROJECT_NAME} PkgConfig::GLIBMM)

target_link_libraries(${PROJECT_NAME} -lrt)

install(TARGETS tflow-process-tracker DESTINATION bin)
