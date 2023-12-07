cmake_minimum_required(VERSION 3.10)

project(autonomy_research_testbed)

#--------------------------------------------------------------
# Define demos

set(DEMOS
    demo_ROS_sensor
    demo_ROS_dART
)

#--------------------------------------------------------------
# Find Chrono

LIST(APPEND CMAKE_PREFIX_PATH "/opt/chrono/lib/cmake/Chrono")
LIST(APPEND CMAKE_PREFIX_PATH "/opt/vsg/lib/cmake/vsgImGui")
LIST(APPEND CMAKE_PREFIX_PATH "/opt/vsg/lib/cmake/vsgXchange")
find_package(Chrono
             COMPONENTS Vehicle Sensor ROS
             OPTIONAL_COMPONENTS Irrlicht VSG
             CONFIG)

if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

#--------------------------------------------------------------
# Misc CMake settings

set(CXX_STANDARD_REQUIRED ON)

# Force C++17, required by ROS Humble
set(CMAKE_CXX_STANDARD 17)

if(APPLE)
    set(CMAKE_MACOSX_BUNDLE ON)
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  if(MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
    add_definitions( "-D_ENABLE_EXTENDED_ALIGNED_STORAGE" )
  endif()
endif()

if(MSVC)
    add_definitions("-D_CRT_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions("-D_SCL_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions( "-DNOMINMAX" )                # do not use MSVC's min/max macros
    add_compile_options(/wd4275)                   # disable warnings triggered by Irrlicht
    add_compile_options(/wd4251)                   # class needs to have dll-interface
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

#--------------------------------------------------------------
# Build the demos

include_directories(${CHRONO_INCLUDE_DIRS})

foreach(PROGRAM ${DEMOS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM} ${PROGRAM}.cpp)

    target_compile_definitions(${PROGRAM} PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"")
    target_compile_options(${PROGRAM} PUBLIC ${CHRONO_CXX_FLAGS})
    target_link_options(${PROGRAM} PUBLIC ${CHRONO_LINKER_FLAGS})

    target_link_libraries(${PROGRAM} ${CHRONO_LIBRARIES})
endforeach()

add_DLL_copy_command()