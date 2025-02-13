# Note that headers are optional, and do not affect add_library, but they will not
# show up in IDEs unless they are listed in add_library.

file(GLOB HEADERS "${PROJECT_SOURCE_DIR}/include/exosim/biomechanics/*.h"
                  "${PROJECT_SOURCE_DIR}/include/exosim/biomechanics/vis/*.h"
                  "${PROJECT_SOURCE_DIR}/include/exosim/biomechanics/physics/*.h"
                  "${PROJECT_SOURCE_DIR}/include/exosim/biomechanics/utils/*.h"
)

file(GLOB SOURCES "${PROJECT_SOURCE_DIR}/src/biomechanics/*.cpp"
                  "${PROJECT_SOURCE_DIR}/src/biomechanics/vis/*.cpp"
                  "${PROJECT_SOURCE_DIR}/src/biomechanics/physics/*.cpp"
                  "${PROJECT_SOURCE_DIR}/src/biomechanics/utils/*.cpp"
)

# Make an automatic library - will be static or dynamic based on user setting
add_library(exosim_biomechanics ${SOURCES} ${HEADERS})
# Alias
add_library(exosim::biomechanics ALIAS exosim_biomechanics)

# We need this directory, and users of our library will need it too
target_include_directories(exosim_biomechanics PUBLIC
        "$<BUILD_INTERFACE:${ExoSim_PROJECT_SOURCE_DIRECTORY}/include/exosim/biomechanics>"
        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/include>"
)

#-----------------------------------------------------------------------------
# === External libraries ===
#-----------------------------------------------------------------------------
include(FetchContent)
set(FETCHCONTENT_QUIET TRUE)

# This depends on:
target_link_libraries(exosim_biomechanics
        PUBLIC ${CHRONO_LIBRARIES}
        PUBLIC fmt::fmt
        PRIVATE mshio::mshio
)

# All users of this library will need at least C++17
target_compile_features(exosim_biomechanics PUBLIC cxx_std_17)

set_target_properties(exosim_biomechanics PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(exosim_biomechanics PROPERTIES DESCRIPTION ${PROJECT_DESCRIPTION})
set_target_properties(exosim_biomechanics PROPERTIES HOMEPAGE_URL ${PROJECT_HOMEPAGE_URL})

string(REPLACE ";" " " CHRONO_CXX_FLAGS_LIST "${CHRONO_CXX_FLAGS}")
string(REPLACE ";" " " CHRONO_LINKER_FLAGS_LIST "${CHRONO_LINKER_FLAGS}")
set_target_properties(exosim_biomechanics PROPERTIES
        COMPILE_FLAGS "${CHRONO_CXX_FLAGS_LIST}"
        LINK_FLAGS "${CHRONO_LINKER_FLAGS_LIST}"
)

# Declare public API of your library. This API will be installed for the third-party application.
# It is a good practice to isolate it in your project tree (like placing it include/ directory).
# Notice that, private headers should not be installed and I strongly suggest to place them with the source files.
set_target_properties(exosim_biomechanics PROPERTIES PUBLIC_HEADER "${HEADERS}")

configure_file(exosim_biomech_lib.pc.in exosim_biomech_lib.pc @ONLY)

# Declare files to install
include(GNUInstallDirs)

install(TARGETS exosim_biomechanics
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/exosim/biomechanics"
)

# IDEs should put the headers in a nice place
source_group(
        TREE "${PROJECT_SOURCE_DIR}"
        PREFIX "Header Files"
        FILES ${HEADERS}
)
