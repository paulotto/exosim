cmake_minimum_required(VERSION 3.18...3.28)
message(STATUS "CMake version: ${CMAKE_VERSION}")

# Project name and a few useful settings. Other commands can pick up the results
project(
        ExoSim
        VERSION 1.0.0
        DESCRIPTION "An extension to the Chrono Project library for simulating biomechanical systems and exoskeletons."
        HOMEPAGE_URL "https://github.com/paulotto/exosim"
        LANGUAGES CXX)
message(STATUS "Project: ${PROJECT_NAME} ${PROJECT_VERSION}")
message(STATUS "${PROJECT_DESCRIPTION}")
message(STATUS "${PROJECT_HOMEPAGE_URL}")


set(USE_MATHJAX NO)
option(BUILD_DOCS "Build documentation" OFF)
option(BUILD_TESTING "Enable testing" ON)

# Only do these if this is the main project, and not if it is included through add_subdirectory
if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

    # Optionally set things like CMAKE_CXX_STANDARD, CMAKE_POSITION_INDEPENDENT_CODE here
    set(CMAKE_CXX_STANDARD 17)

    # Let's ensure -std=c++xx instead of -std=g++xx
    set(CMAKE_CXX_EXTENSIONS OFF)

    # Let's nicely support folders in IDEs
    set_property(GLOBAL PROPERTY USE_FOLDERS ON)

    # Define the option to build shared or static libraries (default: shared)
    option(BUILD_SHARED_LIBS "Build shared libraries instead of static ones" ON)

    # Print information to the user
    if (BUILD_SHARED_LIBS)
        message(STATUS "Building shared libraries")
    else ()
        message(STATUS "Building static libraries")
    endif ()

    # Enable testing
    enable_testing()

    # Docs only available if this is the main app
    if (BUILD_DOCS)
        add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/docs")
    endif ()
endif ()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_definitions(EXOSIM_DEBUG_MODE)
endif ()

# FetchContent added in CMake 3.11, downloads during the configure step
# FetchContent_MakeAvailable was added in CMake 3.14; simpler usage
include(FetchContent)
set(FETCHCONTENT_QUIET TRUE)

# Include ExternalProject module
#include(ExternalProject)

# Third-party libraries
#add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/MshIO")

#-----------------------------------------------------------------------------
# === Boost ===
#-----------------------------------------------------------------------------
#find_package(Boost REQUIRED)

#-----------------------------------------------------------------------------
# === Eigen3 ===
#-----------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)

#-----------------------------------------------------------------------------
# === Chrono package ===
#-----------------------------------------------------------------------------
#LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib")
find_package(Chrono
        COMPONENTS VSG
        OPTIONAL_COMPONENTS PardisoMKL Irrlicht OpenGL # Gpu Multicore Cosimulation Matlab Python Cascade MKL Sensor Modal Postprocess
        CONFIG
)

if (Chrono_FOUND)
    message(STATUS "Chrono found!")
    if (CHRONO_VSG_FOUND)
        message(STATUS "Chrono::VSG found!")
    endif ()
    if (CHRONO_PARDISOMKL_FOUND)
        message(STATUS "Chrono::PardisoMKL found!")
    else ()
        message(STATUS "Chrono::PardisoMKL not found!")
    endif ()
    if (CHRONO_IRRLICHT_FOUND)
        message(STATUS "Chrono::Irrlicht found!")
    else ()
        message(STATUS "Chrono::Irrlicht not found!")
    endif ()
    if (CHRONO_OPENGL_FOUND)
        message(STATUS "Chrono::OpenGL found!")
    else ()
        message(STATUS "Chrono::OpenGL not found!")
    endif ()

    #--------------------------------------------------------------
    # Add path to Chrono headers and to headers of all dependencies
    # of the requested modules.
    #--------------------------------------------------------------

    include_directories(${CHRONO_INCLUDE_DIRS})
else ()
    message(FATAL_ERROR "Could not find Chrono or one of its required modules")
    return()
endif ()

#-----------------------------------------------------------------------------
# === Thirdparty ===
#-----------------------------------------------------------------------------
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/MshIO")

#-----------------------------------------------------------------------------
# === FMT ===
#-----------------------------------------------------------------------------
FetchContent_Declare(
        fmt
        GIT_REPOSITORY https://github.com/fmtlib/fmt.git
        GIT_TAG 10.2.1
)

# Make content available
FetchContent_MakeAvailable(fmt)

# Declare include directory with header files
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/thirdparty")
#include_directories("${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/MshIO/include")
include_directories("${fmt_SOURCE_DIR}/include")

#--------------------------------------------------------------
# Enable creation of "application bundles" on MacOSX.
#--------------------------------------------------------------

# This is necessary for any Irrlicht-based project (like the example here).
# For OpenGL-based or non-graphics projects, this is optional and the block
# below can be removed (or else explcitly set CMAKE_MACOSX_BUNDLE to 'OFF').
#
# If creating application bundles, the build output will be named 'myexe.app'.
# Use the convenience script 'run_app.sh' available under 'contrib/appbundle-macosx/'
# to run:
#     start_demo.sh myexe.app

if (APPLE)
    set(CMAKE_MACOSX_BUNDLE ON)
endif ()

#-----------------------------------------------------------------------------
# Fix for VS 2017 15.8 and newer to handle alignment specification with Eigen
#-----------------------------------------------------------------------------

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    if (MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
        add_definitions("-D_ENABLE_EXTENDED_ALIGNED_STORAGE")
    endif ()
endif ()

#--------------------------------------------------------------
# Tweaks to disable some warnings with MSVC
#--------------------------------------------------------------
if (MSVC)
    add_definitions("-D_CRT_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions("-D_SCL_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions("-DNOMINMAX")                # do not use MSVC's min/max macros
endif ()

# The compiled library code is here
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src/biomechanics")

# Testing only available if this is the main app
# Emergency override MODERN_CMAKE_BUILD_TESTING provided as well
if ((CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME OR MODERN_CMAKE_BUILD_TESTING) AND BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif ()

#--------------------------------------------------------------
# === CONFIG File ===
# Set CMake variables and create a config header file with
# important constants
#--------------------------------------------------------------
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/apps)
#set(CHRONO_SENSOR_SHADER_DIR "$ENV{Chrono_DIR}/include/chrono_sensor/optix/shaders/")

set(RESOURCES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/resources)
# Specify the configuration file from which the header file will be generated
configure_file(ExoSimConfig.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/ExoSimConfig.h @ONLY)

#--------------------------------------------------------------
# === Executables ===
# Add the executable from your project and specify all C++ 
# files in your project. 
#--------------------------------------------------------------
#add_executable(jaw_model src/jaw_model.cpp)
#add_executable(jaw_model_test src/jaw_model_test.cpp)

#add_dependencies(jaw_model exosim::biomechanics)
#add_dependencies(jaw_model_test exosim::biomechanics)

#--------------------------------------------------------------
# Set properties for your executable target
#
# Note that here we define a macro CHRONO_DATA_DIR which will
# contain the path to the Chrono data directory, either in its
# source tree (if using a build version of Chrono), or in its
# install tree (if using an installed version of Chrono).
#--------------------------------------------------------------
set(EXOSIM_COMPILER_FLAGS "${CHRONO_CXX_FLAGS}")
set(EXOSIM_LINKER_FLAGS "${CHRONO_LINKER_FLAGS}")

message(STATUS "ExoSim Compiler flags: ${EXOSIM_COMPILER_FLAGS}")
message(STATUS "ExoSim Linker flags: ${EXOSIM_LINKER_FLAGS}")

set(PROGRAMS
        jaw_model
)

foreach (PROGRAM ${PROGRAMS})
    message(STATUS "...add ${PROGRAM}")

    set(LIBS
            exosim::biomechanics
            ${CHRONO_LIBRARIES}
    )

    add_executable(${PROGRAM} "src/apps/${PROGRAM}.cpp")
    source_group("" FILES "src/apps/${PROGRAM}.cpp")

    target_compile_definitions(${PROGRAM} PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"")

    string(REPLACE ";" " " CHRONO_CXX_FLAGS_LIST "${EXOSIM_COMPILER_FLAGS}")
    string(REPLACE ";" " " CHRONO_LINKER_FLAGS_LIST "${EXOSIM_LINKER_FLAGS}")
    set_target_properties(${PROGRAM} PROPERTIES
            FOLDER apps
            COMPILE_FLAGS "${CHRONO_CXX_FLAGS_LIST}"
            LINK_FLAGS "${CHRONO_LINKER_FLAGS_LIST}")
    if (APPLE)
        set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif ()
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")

    target_link_libraries(${PROGRAM} ${LIBS})

    install(TARGETS ${PROGRAM} DESTINATION ${EXOSIM_INSTALL_APPS})
endforeach (PROGRAM)

#--------------------------------------------------------------
# === (OPTIONAL) ===
#
# Optionally, add a custom command for copying all Chrono and
# dependency DLLs to the appropriate binary output folder.
# This function has effect only on Windows.
#
# DLLs will be copied into ${PROJECT_BINARY_DIR}/${config} by default
# or in ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${config} if only CMAKE_RUNTIME_OUTPUT_DIRECTORY is set
# or to ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_<CONFIG>} if the specific CMAKE_RUNTIME_OUTPUT_DIRECTORY_<CONFIG> has been set
#--------------------------------------------------------------

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "<mycustompathforrelease>")
add_DLL_copy_command()
#add_DLL_copy_command("${CHRONO_DLLS}")
